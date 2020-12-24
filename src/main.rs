#![no_std]
#![no_main]

mod config;
mod can;

use rtic::app;
use cortex_m::asm::delay;
use rtt_target::{rprintln, rtt_init_print};
use cortex_m::interrupt::{free as disable_interrupts, CriticalSection};
use stm32f0xx_hal::time::Hertz;
use stm32f0xx_hal::{
    prelude::*,
    stm32,
    spi::Spi
};

#[app(device = stm32f0xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        can: can::Can
    }
    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        rtt_init_print!();
        let mut dp: stm32::Peripherals = ctx.device;
        let cs = unsafe {CriticalSection::new()};
        let mut rcc = dp.RCC;

        rcc.apb1enr.modify(|_, w| w.canen().enabled()); // can time enb

        let mut clock = rcc
            .configure()
            .sysclk( 48.mhz())
            .freeze(&mut dp.FLASH);

        let gpioa = dp.GPIOA.split(&mut clock);
        let gpiob = dp.GPIOB.split(&mut clock);
        let can_rx: config::CAN_RX_PIN = gpioa.pa11.into_alternate_af4(&cs);
        let can_tx: config::CAN_TX_PIN = gpioa.pa12.into_alternate_af4(&cs);

        let can_params: can::CanParams = can::CanParams{
            work_mode: can::CanMode::NormalMode,
            automatic_retransmission: can::AutomaticRetransmission::Enabled,
            automatic_busoff_management: can::AutomaticBussOffManagement::Enabled,
            auto_wake_up: can::AutomaticWakeUpMode::Enabled,
            pclk_Hz: clock.clocks.pclk(),
            bitrate: can::BitRate::_1Mbs,
        };

        let filter1: can::Filter = can::Filter{
            mode: can::FilterMode::ListMode,
            scale_config: can::FilterScaleConfiguration::_32BitSingleConfig,
            id_or_mask: 0x12345678,
            enable: true,
            id_type: can::IdType::Extended,
            rtr: false
        };

        let filter2: can::Filter = can::Filter{
            mode: can::FilterMode::ListMode,
            scale_config: can::FilterScaleConfiguration::_32BitSingleConfig,
            id_or_mask: 0x11111111,
            enable: true,
            id_type: can::IdType::Extended,
            rtr: false
        };

        let can  = can::Can::new(
            can_tx,
            can_rx,
            dp.CAN,
            can_params,
            &[filter1, filter2]
        );

        init::LateResources {
            can
        }
    }
    #[idle(resources = [can])]
    fn idle(ctx: idle::Context) -> ! {
        let mut can = ctx.resources.can;
        loop {
            delay(6_000_000);
            rprintln!("*************************");
            delay(6_000_000);
        }

    }

    #[task(binds = CEC_CAN, priority = 4 , resources = [can])]
    fn can_irq(ctx: can_irq::Context){
        let can: &mut can::Can = ctx.resources.can;
        can.irq_state_machine(|id, data|{
            rprintln!("CAN_IRQ: id: {:x}; Data: {:?}", id, data);
        });
    }
};


use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use nb::Error;
use core::borrow::{BorrowMut, Borrow};


#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("Panic: {:?}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}