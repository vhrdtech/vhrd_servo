#![no_std]
#![no_main]

//extern crate panic_halt;
//use cortex_m::asm;
use cortex_m_rt::entry;

use jlink_rtt;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use numtoa::NumToA;

use cortex_m::peripheral::Peripherals;
use stm32f0xx_hal as hal;
use crate::hal::{
    delay::Delay,
    prelude::*,
    stm32
};

#[entry]
fn main() -> ! {
    let mut output = jlink_rtt::NonBlockingOutput::new(false);
    //let mut output = jlink_rtt::Output::new();
    let _ = output.write("vhrd-micro-servo ");
    let _ = output.write(env!("CARGO_PKG_VERSION"));
    let _ = output.write("\r\n");

    if let (Some(mut p), Some(cp)) = (stm32::Peripherals::take(), Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
            let gpioa = p.GPIOA.split(&mut rcc);
            let gpiob = p.GPIOB.split(&mut rcc);
            let gpiof = p.GPIOF.split(&mut rcc);
            let mut delay = Delay::new(cp.SYST, &rcc);

            let mut buf = [0_u8; 8];


            let can = p.CAN;
            let mut tja1042_standby_pin = gpioa.pa6.into_push_pull_output(&cs);
            tja1042_standby_pin.set_low();

            unsafe {
                // Enable CAN & SYSCFG clock
                let mut rcc = stm32f0xx_hal::stm32::RCC::ptr();

                (*rcc).apb2enr.modify(|_, w| w.syscfgen().set_bit());
                // Use PA11/12 instead of 9/10
                p.SYSCFG.cfgr1.modify(|_, w| w.pa11_pa12_rmp().set_bit());
                (*rcc).apb1enr.modify(|_, w| w.canen().set_bit());

                let mut gpioa = stm32f0xx_hal::stm32::GPIOA::ptr();
                let mut gpioa = &(*gpioa);
                gpioa.moder.modify(|_, w| w.moder11().alternate().moder12().alternate());
                gpioa.afrh.modify(|_, w| w.afrh11().af4().afrh12().af4());
                //gpioa.otyper.modify(|_, w| w.ot11().clear_bit().ot12().clear_bit());
                //gpioa.ospeedr.modify(|_, w| w.ospeedr11().very_high_speed().ospeedr12().very_high_speed()); // ?
                //gpioa.pupdr.modify(|_, w| w.pupdr11().floating().pupdr12().floating());

                let rxpin: u8 = gpioa.idr.read().idr11().bit_is_set().into();
                let _ = output.write("rx pin=");
                let _ = output.write_bytes(rxpin.numtoa(10, &mut buf));
                let _ = output.write("\n");

                // Leave sleep mode
                can.mcr.modify(|_, w| w.sleep().clear_bit());
                // Wait sleep leave ack
                let mut timeout = 1_000_000_u32;
                while can.msr.read().slak().bit_is_set() {
                    timeout = timeout - 1;
                    if timeout == 0 {
                        let _ = output.write("w1 fail\n");
                        break;
                    }
                }
                // Request initialization
                can.mcr.modify(|_, w| w.inrq().set_bit());
                // Wait until CAN is in init mode
                timeout = 1_000_000_u32;
                while can.msr.read().inak().bit_is_clear() {
                    timeout = timeout - 1;
                    if timeout == 0 {
                        let _ = output.write("w3 fail\n");
                        break;
                    }
                }

                // Configure timings
                // 1/24mhz=41.67ns
                // bittime=2us for 500kbps
                // 12tq per bittime
                // 166.67ns per tq
                can.btr.write(|w| w.brp().bits(4).ts1().bits(7).ts2().bits(2)); // Timings

                // Configure mode
                can.mcr.modify(|_, w| w.ttcm().clear_bit()
                    .awum().set_bit()   // leave sleep mode on CAN message detection
                    .abom().set_bit()   // bus-off is left automatically after 128*11 recessive bits
                    .dbf().clear_bit()  // continue working under debug
                    .nart().set_bit()   // disable retransmission
                    .ttcm().clear_bit() // disable time triggered mode
                    .rflm().clear_bit() // receive FIFO not locked on overrun
                    .txfp().clear_bit() // priority driven by identifier of the message
                );

                // Request normal mode
                can.mcr.modify(|_, w| w.inrq().clear_bit());
                // Wait until CAN is in init mode
                timeout = 1_000_000_u32;
                while can.msr.read().inak().bit_is_set() {
                    timeout = timeout - 1;
                    if timeout == 0 {
                        let _ = output.write("w4 fail\n");
                        break;
                    }
                }

                // Filter init mode
                can.fmr.modify(|_, w| w.finit().set_bit());
                // Disable filter 0
                can.fa1r.modify(|_, w| w.fact0().clear_bit());
                // Single 32-bit filter
                can.fs1r.modify(|_, w| w.fsc0().set_bit());
                can.fb[0].fr1.write(|w| w.bits(0));
                can.fb[0].fr2.write(|w| w.bits(0));
                // Enable filter 0
                can.fa1r.modify(|_, w| w.fact0().set_bit());
                // Filter active mode
                can.fmr.modify(|_, w| w.finit().clear_bit());
            }






            let mut drv_en = gpiob.pb1.into_push_pull_output(cs);
            let mut drv_ph = gpiob.pb0.into_push_pull_output(cs);
            let mut drv_sleep = gpioa.pa7.into_push_pull_output(cs);

            let mut angle_in = gpioa.pa3.into_analog(cs);
            let mut adc = hal::adc::Adc::new(p.ADC, &mut rcc);

            let mut led = gpioa.pa4.into_push_pull_output(cs);

            drv_sleep.set_high();



            loop {
                let angle_in_raw: u16 = adc.read(&mut angle_in).unwrap();

                let _ = output.write("\x1b[2J\x1b[1;1H");
                let _ = output.write("angle_in: ");
                let _ = output.write("\x1b[31m");
                let _ = output.write_bytes(angle_in_raw.numtoa(10, &mut buf));
                let _ = output.write("\x1b[0m\r\n");

                let esr = can.esr.read();
                let receive_error_counter = esr.rec().bits();
                let transmit_error_counter = esr.tec().bits();
                let last_error_code = esr.lec().bits();
                let bus_off = esr.boff().bit();
                let error_passive_flag = esr.epvf().bit();
                let error_warning_flag = esr.ewgf().bit();
                let msr = can.msr.read();
                let rx_pin = msr.rx().bit();
                let receive_mode = msr.rxm().bit();
                let transmit_mode = msr.txm().bit();
                let error_interrupt = msr.erri().bit();
                let _ = output.write("rec: ");
                let _ = output.write_bytes(receive_error_counter.numtoa(10, &mut buf));
                let _ = output.write("\ttec: ");
                let _ = output.write_bytes(transmit_error_counter.numtoa(10, &mut buf));
                let _ = output.write("\tlec: ");
                match last_error_code {
                    0b000 => output.write("no_err"),
                    0b001 => output.write("stuff_err"),
                    0b010 => output.write("form_err"),
                    0b011 => output.write("ack_err"),
                    0b100 => output.write("bitr_err"),
                    0b101 => output.write("bitd_err"),
                    0b110 => output.write("crc_err"),
                    0b111 => output.write("soft_err"),
                    _ => {}
                }
                if bus_off {
                    let _ = output.write("\tbus_off");
                }
                if error_passive_flag {
                    let _ = output.write("\tepvf");
                }
                if error_warning_flag {
                    let _ = output.write("\tewgf");
                }
                if rx_pin {
                    let _ = output.write("\trxp=1");
                }
                if receive_mode {
                    let _ = output.write("\trxm");
                }
                if transmit_mode {
                    let _ = output.write("\ttxm");
                }
                if error_interrupt {
                    let _ = output.write("\terri");
                }


                let n = jlink_rtt::try_read(&mut buf);
                if n > 0 {
                    if buf[0] == 'f' as u8 {
                        drv_ph.set_high();
                        //let _ = output.write("f++\r\n");
                    } else if buf[0] == 'r' as u8 {
                        drv_ph.set_low();
                        //let _ = output.write("r++\r\n");
                    } else if buf[0] == 's' as u8 {

                    } else {
                        let _ = output.write("?\r\n");
                    }
                    drv_en.set_high();

                    led.set_high();
                    delay.delay_ms(10_u16);
                    led.set_low();

                    drv_en.set_low();
                }
                delay.delay_ms(20_u16);
            }
        });
    }

    loop {

    }
}

#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    let mut output = jlink_rtt::NonBlockingOutput::new(false);
    let payload = panic_info.payload().downcast_ref::<&str>();
    let mut buffer = [0_u8; 8];
    match (panic_info.location(), payload) {
        (Some(location), Some(msg)) => {
            let _ = output.write("\r\npanic in file ");
            let _ = output.write(location.file());
            let _ = output.write(" at line ");
            let line = location.line();
            let _ = output.write_bytes(line.numtoa(10, &mut buffer));
            let _ = output.write(msg);
        }
        (Some(location), None) => {
            let _ = output.write("\r\npanic in file ");
            let _ = output.write(location.file());
            let _ = output.write(" at line ");
            let line = location.line();
            let _ = output.write_bytes(line.numtoa(10, &mut buffer));
        }
        (None, Some(msg)) => {
            let _ = output.write("\r\npanic ");
            let _ = output.write(msg);
        }
        (None, None) => {
            let _ = output.write("panic occured, no info available");
        }
    }
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}