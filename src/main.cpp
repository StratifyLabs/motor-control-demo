#include <stdio.h>

#include <sapi/hal.hpp>
#include <sapi/calc.hpp>
#include <sapi/chrono.hpp>
#include <sapi/var.hpp>
#include <sapi/sys.hpp>

//TIM2 is system timer
//TIM4_CH4 is Pin 18 and CN7 (PD15) D9
//TIM14_CH1 is Pin 14 and CN7 (PA7 or PB5)
//TIM3_CH3 is Pin 31 and CN10 (PB0)

int main(int argc, char * argv[]){
	Tmr pwm_timer(3); //Use TIM4 CH4 -- ports are 0,1,2,3 for TIM1, TIM2, TIM3, TIM4
	TmrAttributes pwm_timer_attributes;

	const u32 pmw_channel_number = 3; //CH1 -> 0, CH2 -> 1 ... CH4 -> 3
	const u32 pwm_timer_period = 1000;
	const u32 pwm_timer_frequency = 100000;

	pwm_timer_attributes.set_flags(
				Tmr::SET_TIMER |
				Tmr::IS_SOURCE_CPU |
				Tmr::IS_AUTO_RELOAD |
				Tmr::SET_CHANNEL |
				Tmr::IS_CHANNEL_PWM_MODE
				)
			.set_period(pwm_timer_period)
			.set_frequency(pwm_timer_frequency)
			.set_channel(
				arg::Location(pmw_channel_number),
				arg::Value(0)
				)
			.assign_pin( //assign PD15 to use as a channel
				arg::Position(0), //use slot zero to assign this pin
				arg::PortNumber(3), //PORTA -> 0 ... PORTD -> 3
				arg::PinNumber(15) //Pin 15
				);


	printf("Initialize PWM timer\n");
	pwm_timer.initialize(
				pwm_timer_attributes
				);

	if( pwm_timer.return_value() < 0 ){
		printf(
					"Failed to initialize PWM timer (%d, %d)\n",
					pwm_timer.return_value(),
					pwm_timer.error_number()
				 );
		exit(1);
	}

	pwm_timer.enable();

	printf("wait 5 seconds %ld\n", pwm_timer.get_value());
	wait(Seconds(5));
	printf("exit %ld\n", pwm_timer.get_value());

	exit(1);

	Tmr encoder_timer(2); //use TIM3_CH3 for encoder input 0->TIM1, 1->TIM2, 2->TIM3
	TmrAttributes encoder_timer_attributes;


	encoder_timer.initialize( encoder_timer_attributes );
	if( encoder_timer.return_value() < 0 ){
		printf(
					"Failed to initialize encoder timer (%d, %d)",
					encoder_timer.return_value(),
					encoder_timer.error_number()
					);
		exit(1);
	}

	//calculations
	PidF32 pid_control;
	pid_control
			.set_kp(1.0f)
			.set_ki(0.1f)
			.set_kd(0.001f)
			.set_maximum(1.0f) //max duty cycle
			.set_minimum(0.0f); //min duty cycle

	Adc poteniometer_input(0);
	AdcAttributes adc_attributes;

	poteniometer_input.initialize( adc_attributes );
	if( poteniometer_input.return_value() < 0 ){
		printf(
					"Failed to initialize ADC (%d, %d)\n",
					poteniometer_input.return_value(),
					poteniometer_input.error_number()
					);
	}

	LowPassFilterF32 poteniometer_filter(0.0f, 0.1f);


	Vector<u16> poteniometer_samples(arg::Count(64));
	Timer loop_timer;
	u32 last_count_value = 0;
	u32 current_count_value = 0;
	float target_speed;
	float current_speed;
	float duty_cycle;

	while(1){

		//read 64 ADC samples
		poteniometer_input.read(
					arg::DestinationData(poteniometer_samples)
					);

		//smooth out the ADC input
		for(auto sample: poteniometer_samples){
			poteniometer_filter << sample;
		}

		//convert the ADC input to the PID target set point
		target_speed = 0.0f;

		pid_control.set_target(
					target_speed
					);

		//calculate the speed
		current_count_value = encoder_timer.get_value();
		current_speed = (current_count_value - last_count_value)/loop_timer.milliseconds();
		loop_timer.restart();
		last_count_value = current_count_value;

		duty_cycle = pid_control.calculate_control_variable(
					current_speed
					);

		//apply the new duty cycle to the PWM
		pwm_timer.set_channel(
					arg::Location(0),
					arg::Value(pwm_timer_period * duty_cycle)
					);

		chrono::wait( Milliseconds(25) );

	}





	return 0;
}

