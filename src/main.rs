#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![macro_use]
extern crate alloc;

use core::mem::MaybeUninit;

use alloc::boxed::Box;
use esp_backtrace as _;
use esp_println::println;

use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyleBuilder},
    text::{Baseline, Text, TextStyleBuilder},
};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{SpiBus, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::{
    clock::{ClockControl, Clocks},
    dma::DmaPriority,
    entry,
    gdma::Gdma,
    gpio::{GpioPin, Input, Output, PullDown, PushPull, IO},
    peripherals::{Peripherals, SPI2},
    prelude::*,
    spi::master::dma::SpiDma,
    spi::{
        master::{prelude::*, Spi, SpiBusController, SpiBusDevice},
        FullDuplexMode, SpiMode,
    },
    systimer::SystemTimer,
    timer::TimerGroup,
    Delay, Rng,
};

use epd_waveshare::{
    epd7in5b_v2::{Display7in5, Epd7in5, HEIGHT as DISPLAY_HEIGHT, WIDTH as DISPLAY_WIDTH},
    graphics::VarDisplay,
    prelude::*,
};
use static_cell::make_static;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 128 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}


#[entry]
fn main() -> ! {
    init_heap();

    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // let ledpin = gpio.gpio2;
    // let wakeup_pin = gpio.gpio4;

    let sclk = io.pins.gpio12.into_push_pull_output();
    let mosi = io.pins.gpio11.into_push_pull_output();
    let cs = io.pins.gpio10.into_push_pull_output();

    let busy = io.pins.gpio18.into_pull_up_input();
    let rst = io.pins.gpio16.into_push_pull_output();
    let dc = io.pins.gpio17.into_push_pull_output();

    // NON-DMA VERSION
    //------------------------------------------------------------------------
    // let spi = Spi::new_no_cs_no_miso(
    //     peripherals.SPI2,
    //     sclk,
    //     mosi,
    //     2u32.MHz(),
    //     SpiMode::Mode0,
    //     &clocks,
    // );

    // log::info!("Locking spi bus");

    // let spi_controller = SpiBusController::from_spi(spi);
    // let mut spi_device = spi_controller.add_device(cs);

    // NON-DMA VERSION 2
    //------------------------------------------------------------------------
    // let spi = Spi::new_no_cs_no_miso(
    //     peripherals.SPI2,
    //     sclk,
    //     mosi,
    //     2u32.MHz(),
    //     SpiMode::Mode0,
    //     &clocks,
    // );

    // log::info!("Locking spi bus");

    // let mut spi_device = ExclusiveDevice::new(spi, cs, delay);

    // DMA VERSION
    //------------------------------------------------------------------------
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_IN_CH0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::DMA_OUT_CH0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut tx_descriptors = [0u32; 8 * 6];
    let mut rx_descriptors = [0u32; 8];

    let mut spi_bus = Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        sclk,
        mosi,
        2u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    )
    .with_dma(dma_channel.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    log::info!("Locking spi bus");

    let mut delay = Delay::new(&clocks);
    let mut spi_device = ExclusiveDevice::new(spi_bus, cs, delay);

    // Setup the epd
    let mut delay = Delay::new(&clocks);

    let mut epd = Epd7in5::new(&mut spi_device, busy, dc, rst, &mut delay, None).unwrap();

    // Setup the graphics
    // Display uses #[inline(always)] so stack allocation should be skipped here
    let mut disp = Box::new(Display7in5::default());

    let display = disp.as_mut();

    println!("Now test new graphics with default rotation and some special stuff");
    display.clear(TriColor::White).ok();

    // draw a analog clock
    let style = PrimitiveStyleBuilder::new()
        .stroke_color(TriColor::Black)
        .stroke_width(1)
        .build();

    let _ = Circle::with_center(Point::new(64, 64), 80)
        .into_styled(style)
        .draw(display);
    let _ = Line::new(Point::new(64, 64), Point::new(0, 64))
        .into_styled(style)
        .draw(display);
    let _ = Line::new(Point::new(64, 64), Point::new(80, 80))
        .into_styled(style)
        .draw(display);

    // draw white on black background
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_6X10)
        .text_color(TriColor::Chromatic)
        .background_color(TriColor::Black)
        .build();
    let text_style = TextStyleBuilder::new().baseline(Baseline::Top).build();

    let _ = Text::with_text_style("It's working-WoB!", Point::new(175, 250), style, text_style)
        .draw(display);

    // use bigger/different font
    let style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
        .text_color(TriColor::White)
        .background_color(TriColor::Black)
        .build();

    let _ = Text::with_text_style("It's working-WoB!", Point::new(50, 200), style, text_style)
        .draw(display);

    let mut delay = Delay::new(&clocks);

    println!("Draw!");

    // Transfer the frame data to the epd and display it
    epd.update_and_display_frame(&mut spi_device, &display.buffer(), &mut delay)
        .unwrap();

    println!("wait for idle!");
    epd.wait_until_idle(&mut spi_device, &mut delay).unwrap();

    println!("sleep!");
    epd.sleep(&mut spi_device, &mut delay).unwrap();

    loop {
        println!("Loop...");
        delay.delay_ms(500u32);
    }
}
