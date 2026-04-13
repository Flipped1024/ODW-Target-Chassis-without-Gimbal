#include "cap.h"

cap_t Cap;
void Cap_unused_correct(void)
{
    // static int count=0;
    // if(Cap.Cap_Normol_Chassis_Flag==1)
    //     count++;
    // else
    //     count=0;

    // if(count>=1000)
    //     Power_Control.Is_Cap_Used=FALSE;
    // else
    //     Power_Control.Is_Cap_Used=TRUE;

    // if(is_TOE_Error(CAP_TOE))
    //     Power_Control.Is_Cap_Used=FALSE;  
    
    Power_Control.Is_Cap_Used=FALSE;
}