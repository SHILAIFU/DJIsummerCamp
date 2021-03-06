typedef enum
{
    m100_INIT               ,
    m100_TAKEOFF            ,
    m100_LANDING            ,
    m100_RELEASE_CONTROL    ,
    m100_STAND_BY           ,
    REACH_ALT          ,
    COLLECTING_HIPPO   ,
    RELEASING_HIPPO    ,
    RELEASING_OCTOPUS  ,
    ZONE2_BOX_AIMING   ,
    ZONE3_BOX_AIMING   ,
    ZONE4_BOX_AIMING   ,
    START_SEARCH       ,
    START_TRACK        ,
    VISION_START       ,
    LOCK_TARGET        ,
    TRACKING           ,
    LOCK_HOOK          ,
    RELEASE_HOOK       ,
    PLACE_BARREL       ,
    SEARCH_DESTINATION ,
    AIM_ARUCO          ,
    GO_BACK_TO_SOURCE  ,
    CLR                ,
    END_PROCESS        ,

    RETURN_HOME        ,
    MISSION_PROCESSING ,
    MISSION_ABORTED    ,
    MISSION_COMPLETED  ,
    EMERGENCY          ,

} MISSION_STATUS;


