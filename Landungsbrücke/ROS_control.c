// use TMC4671 API

#include "tmc/ic/TMC4671/TMC4671.h"

#include "boards/Board.h"
#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/IdDetection.h"
#include "tmc/TMCL.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/BoardAssignment.h"



const char *VersionString = MODULE_ID"V307"; // module id and version of the firmware shown in the TMCL-IDE

/* Keep as is! This lines are important for the update functionality. */
#if defined(Landungsbruecke)
	const uint8_t Protection[] __attribute__ ((section(".cfmconfig")))=
	{
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //Backdoor key
		0xFF, 0xFF, 0xFF, 0xFF,                          //Flash protection (FPPROT)
		0x7E,                                            //Flash security   (FSEC) => nach Image-Generierung manuell auf 0x40 setzen im Image
		0xF9,                                            //Flash option     (FOPT) (NMI ausgeschaltet, EzPort ausgeschaltet, Normal power)
		0xFF,                                            //reserved
		0xFF                                             //reserved
	};

	struct BootloaderConfig {
		uint32_t BLMagic;
		uint32_t drvEnableResetValue;
	};

	// This struct gets placed at a specific address by the linker
	struct BootloaderConfig __attribute__ ((section(".bldata"))) BLConfig;
#endif

	void shallForceBoot()
	{
		// toggle each pin and see if you can read the state on the other
		// leave if not, because this means that the pins are not tied together
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
		HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

		HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK);
		if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
			return;

		HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK);
		if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
			return;

		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
		HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CLK);

		HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CH0);
		if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
			return;

		HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CH0);
		if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
			return;

		// not returned, this means pins are tied together
		tmcl_boot();
	}

void init_reg(){

#if defined(Landungsbruecke)
	// Default value: Driver enable gets set high by the bootloader
	BLConfig.drvEnableResetValue = 1;
#endif

	HAL.init();                  // Initialize Hardware Abstraction Layer
	IDDetection_init();          // Initialize board detection
	tmcl_init();                 // Initialize TMCL communication

	tmcdriver_init();            // Initialize dummy driver board --> preset EvalBoards.ch2
	tmcmotioncontroller_init();  // Initialize dummy motion controller board  --> preset EvalBoards.ch1

	VitalSignsMonitor.busy = 1;  // Put state to busy
	Evalboards.driverEnable = DRIVER_ENABLE;
	Evalboards.ch1.id = 0;       // preset id for driver board to 0 --> error/not found
	Evalboards.ch2.id = 0;       // preset id for driver board to 0 --> error/not found

	// We disable the drivers before configurating anything
	HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO0);
	HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO0);

	IdAssignmentTypeDef ids;
	IDDetection_initialScan(&ids);  // start initial board detection
	IDDetection_initialScan(&ids);  // start second time, first time not 100% reliable, not sure why - too fast after startup?
	if(!ids.ch1.id && !ids.ch2.id)
	{
		shallForceBoot();           // only checking to force jump into bootloader if there are no boards attached
		// todo CHECK 2: Workaround: shallForceBoot() changes pin settings - change them again here, since otherwise IDDetection partially breaks (LH)
		HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
		HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);
	}
	Board_assign(&ids);             // assign boards with detected id

	VitalSignsMonitor.busy 	= 0;    // not busy any more!

}


void init_motor(){

	// Motor type &  PWM configuration
	tmc4671_writeInt(0, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030004);
	tmc4671_writeInt(0, TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(0, TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(0, TMC4671_PWM_BBM_H_BBM_L, 0x00000A0A);
	tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000007);

	// ADC configuration
	tmc4671_writeInt(0, TMC4671_ADC_I_SELECT, 0x18000100); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_A, 0x20000000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_B, 0x00000000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ADC_I0_SCALE_OFFSET, 0x0100815B); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ADC_I1_SCALE_OFFSET, 0x010081CE); // @suppress("Invalid arguments")

	// ABN encoder settings
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_MODE, 0x00000000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_PPR, 0x00001000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00000CAD); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000); // @suppress("Invalid arguments")

	// Limits
	tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x00000BB8); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_PID_VELOCITY_LIMIT, 0x00000064);
	// PI settings
	tmc4671_writeInt(0, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_PID_VELOCITY_P_VELOCITY_I, 0x271005DC);
	tmc4671_writeInt(0, TMC4671_PID_POSITION_P_POSITION_I, 0x000F0002);

}


void start_loop(){

	// ===== ABN encoder test drive =====

	// Init encoder (mode 0)
	tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000001); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_PHI_E_EXT, 0x00000000); // @suppress("Invalid arguments")
	tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0x000007D0);
	wait(1000);
	tmc4671_writeInt(0, TMC4671_ABN_DECODER_COUNT, 0x00000000);

	// Feedback selection
	tmc4671_writeInt(0, TMC4671_PHI_E_SELECTION, 0x00000003);
	tmc4671_writeInt(0, TMC4671_VELOCITY_SELECTION, 0x00000009);

	// Switch to position mode
	tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000003);

	//uint8_t time = 1;
	uint8_t pos_target[4];
	uint8_t enc[4];
	uint32_t target;
	float current_pos;
	while(1){

		current_pos = tmc4671_readInt(0, TMC4671_PID_POSITION_ACTUAL);
		//current_pos = (current_pos/262144) * 360  ;   // 2^16 represent PI/2 radians
		enc[0] = ( current_pos & 0xFF000000 ) >> 24;
		enc[1] = ( current_pos & 0x00FF0000 ) >> 16;
		enc[2] = ( current_pos & 0x0000FF00 ) >> 8;
		enc[3] = ( current_pos & 0x000000FF ) ;

		HAL.USB->txN(enc,4);	//transmit position as 4 bytes in big endian format

		if(HAL.USB->rxN(a,4)){			//receive position as 4 bytes in big endian format
			target = ( a[0] << 24 ) + ( a[1] << 16 ) + ( a[2] << 8 ) + a[3];  
		//	target = (target/360) * 262144;
			tmc4671_writeInt(0, TMC4671_PID_POSITION_TARGET, target);  
		}

		wait(10);
	}
	// Stop
	tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001); // switch to torque mode

	tmc4671_writeInt(0, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000); //set torque target to 0

}

int main(void) {


	init_reg();
	init_motor();
	start_loop();
	
	return 0;
}