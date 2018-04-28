#include "config.h"
#include "spi.h"
#include "stmflash.h"
#include "MoveBase.h"
#include "I2C.h"

int main(void)
{	
	mySystem_init();
	CMoveBase_setup( &MoveBase );
	while (1)
	{
		CMoveBase_loop( &MoveBase );
	}
}
