MODULE Motion_Procs
    
    VAR TCPIP_MSG_COMMAND act_tcpip_message_command; !STRUC command
    
    CONST speeddata fast:=[400,30,5000,1000];
    CONST speeddata mid:=[200,12,5000,1000];
    CONST speeddata slow:=[50,3,5000,1000];
    
    CONST num acc_max:=2;
    CONST num dec_max:=0.2;
    
    VAR speeddata current_speed_data:=slow;
    VAR zonedata current_zone_data:=z10;
    
    !for robtarget moves
    VAR robtarget current_pos;
    VAR bool joint_reach;
    
    !for joint moves
    VAR jointtarget current_jointpos;
    VAR errnum errnum_jointpos;
    
    !general
    VAR robtarget init_pos;
    
    !reset with variable
    VAR intnum rtm_interrupt;

    PROC init()
        !initialize robot
        TPWrite "initialization";
        
        current_tool:=tool0;     
        current_pos:=CRobT(\Tool:=current_tool \WObj:=wobj0);
        init_pos:=CRobT(\Tool:=current_tool \WObj:=wobj0);
        current_jointpos:=CJointT();
        
        joint_reach:=TRUE; 
 
        cmd_exec_counter:=0; !waypoint_counter_exec:=0; !waypoint counter of executed commands
        
        ! limit path acceleration for every move command but the move joints command sent with duration
        PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;
        
        !reset (connected to persistent)
        reset_to_main:=FALSE;
        CONNECT rtm_interrupt WITH reset_after_connection_stop;
        IPers reset_to_main, rtm_interrupt;
        
    ENDPROC
    
    !reset trap (connected to persistent)
    TRAP reset_after_connection_stop
        
        IF reset_to_main = TRUE THEN
            
            TPWrite "INTERRUPT IN MOTION, PP is moved to main";
            
            Idelete rtm_interrupt;
            reset_to_main:=FALSE;
            
            ExitCycle;
            
        ENDIF
        
    ENDTRAP
    
    !reset restart event (connected to persistent and restart)
    PROC reset_after_connection_stop_evt()
        
        IF reset_to_main = TRUE THEN
            
            TPWrite "RESTART EVENT: RESET IN MOTION, PP is moved to main";
            
            Idelete rtm_interrupt;
            reset_to_main:=FALSE;
            
            ExitCycle;
            
        ENDIF
        
    ENDPROC
    
    PROC execute_from_buffer()
        
        act_tcpip_message_command:=tcpip_message_command_buffer{READ_PTR};
        TPWrite "EXECUTE FROM BUFFER" + ValToStr(act_tcpip_message_command.cmd_type);
        
        set_tool;
        set_zone_data;

        TEST act_tcpip_message_command.cmd_type
        
            CASE CMD_GO_TO_JOINTTARGET_ABS:
                read_jointtarget_values_abs;
                move_from_buffer_joint;
            CASE CMD_GO_TO_JOINTTARGET_REL:
                read_jointtarget_values_rel;
                move_from_buffer_joint;
            CASE CMD_GO_TO_TASKTARGET:!move from buffer in task space
                set_speed;
                read_robtarget_values;
                move_from_buffer_robtarget;
            CASE CMD_PICK_BRICK:!pick up fullbrick
                set_speed;
                read_jointtarget_values_abs;
                pick_up_brick_from_feed;
            CASE CMD_PLACE_BRICK:
                set_speed;
                read_robtarget_values;
                place_brick_at_pose;
            CASE CMD_PICK_BRICK_FROM_POSE:
                set_speed;
                read_robtarget_values;
                pick_up_brick_from_pose;
            DEFAULT:
                TPWRITE "Wrong Procedure Number";
        ENDTEST
        
        WaitTestAndSet msg_lock;
            READ_PTR:=(READ_PTR MOD MAX_BUFFER_SIZE) + 1;
            BUFFER_LENGTH:=BUFFER_LENGTH - 1;
        msg_lock:=FALSE;
        
    ENDPROC
    
    LOCAL PROC set_tool()   
        !assign toolnumber
        TEST act_tcpip_message_command.tool
            CASE 0:
                current_tool:=tool0;
            CASE 1:
                !current_tool:=vakgrip_angle_150821_m1;
                current_tool:=mm_tool2;
            CASE 2:
                current_tool:=mm_tool2_with_bending_pin;
            CASE 3:
                current_tool:=mtip_tool2;
            DEFAULT:
                TPWrite "WRONG TOOLNUMBER";
                current_tool:=tool0;
        ENDTEST
    ENDPROC
    
    LOCAL PROC set_speed()
        !assign speedvalue
        TEST act_tcpip_message_command.velocity
            CASE 0:
                current_speed_data:=slow;
            CASE 1:
                current_speed_data:=mid;
            CASE 2:
                current_speed_data:=fast; 
            DEFAULT:
                TPWrite "WRONG SPEEDNUMBER";
                current_speed_data:=slow;
        ENDTEST
    ENDPROC     
    
    LOCAL PROC set_zone_data()
        !assign speedvalue
        TEST act_tcpip_message_command.zone
            CASE 0:
                current_zone_data:=fine;
            DEFAULT:
                current_zone_data:=[FALSE,act_tcpip_message_command.zone,1.5*act_tcpip_message_command.zone,1.5*act_tcpip_message_command.zone,0.15*act_tcpip_message_command.zone,1.5*act_tcpip_message_command.zone,0.15*act_tcpip_message_command.zone];
        ENDTEST
    ENDPROC
    
    LOCAL PROC read_robtarget_values()        
        !read cartesian coordinates
        current_pos.trans.x:=act_tcpip_message_command.val1;
        current_pos.trans.y:=act_tcpip_message_command.val2;
        current_pos.trans.z:=act_tcpip_message_command.val3;
        current_pos.rot.q1:=act_tcpip_message_command.val4;
        current_pos.rot.q2:=act_tcpip_message_command.val5;
        current_pos.rot.q3:=act_tcpip_message_command.val6;
        current_pos.rot.q4:=act_tcpip_message_command.val7; 
    ENDPROC
    
    LOCAL PROC read_jointtarget_values_rel()
        !read jointtarget and add to current jointposition
        current_jointpos:=CJointT(); 
        current_jointpos.robax.rax_1:=current_jointpos.robax.rax_1 + act_tcpip_message_command.val1;
        current_jointpos.robax.rax_2:=current_jointpos.robax.rax_2 + act_tcpip_message_command.val2;
        current_jointpos.robax.rax_3:=current_jointpos.robax.rax_3 + act_tcpip_message_command.val3;
        current_jointpos.robax.rax_4:=current_jointpos.robax.rax_4 + act_tcpip_message_command.val4;
        current_jointpos.robax.rax_5:=current_jointpos.robax.rax_5 + act_tcpip_message_command.val5;
        current_jointpos.robax.rax_6:=current_jointpos.robax.rax_6 + act_tcpip_message_command.val6;
    ENDPROC
    
    LOCAL PROC read_jointtarget_values_abs()
        current_jointpos.robax.rax_1:=act_tcpip_message_command.val1;
        current_jointpos.robax.rax_2:=act_tcpip_message_command.val2;
        current_jointpos.robax.rax_3:=act_tcpip_message_command.val3;
        current_jointpos.robax.rax_4:=act_tcpip_message_command.val4;
        current_jointpos.robax.rax_5:=act_tcpip_message_command.val5;
        current_jointpos.robax.rax_6:=act_tcpip_message_command.val6;
    ENDPROC

    PROC move_from_buffer_joint()       
        !TPWrite "=> Start moving to pos from buffer. buflen: " + ValToStr(BUFFER_LENGTH);

        TEST act_tcpip_message_command.duration
            CASE 0:
                set_speed;
                PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;
                
                IF BUFFER_LENGTH > 1 THEN
                    MoveAbsJ current_jointpos,current_speed_data,current_zone_data,tool0;
                ELSE
                    MoveAbsJ current_jointpos,current_speed_data,fine,tool0;
                ENDIF
                
            DEFAULT:
                IF BUFFER_LENGTH > 1 THEN
                    PathAccLim FALSE,FALSE;
                    MoveAbsJ current_jointpos,current_speed_data\T:=act_tcpip_message_command.duration,current_zone_data,tool0;
                    PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;
                ELSE
                    MoveAbsJ current_jointpos,current_speed_data\T:=act_tcpip_message_command.duration,current_zone_data,tool0;
                ENDIF    
        ENDTEST
        
        ! to check when a trajectory end is reached or to confirm execution of a single joint target.
        IF (current_zone_data = fine)  OR (act_tcpip_message_command.arbitrary=1) THEN
            send_command_executed;
        ENDIF
    ENDPROC
    
    PROC move_from_buffer_robtarget()
        !TPWrite "=> Start moving to pos from buffer. buflen: " + ValToStr(BUFFER_LENGTH);

        !joint_reach:=calc_joint_reach(current_pos); !calc reachability
        
        !SingArea \Off; 
        SingArea \Wrist;
        ConfL \Off;
        PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;
        
        IF joint_reach = TRUE THEN
            IF BUFFER_LENGTH > 1 THEN
                MoveLSync current_pos,current_speed_data,current_zone_data,current_tool,\WObj:=wobj0,"send_command_executed";
            ELSE
                MoveL current_pos,current_speed_data,fine,current_tool,\WObj:=wobj0;
                send_command_executed;
            ENDIF
        ELSE
           send_command_executed; 
        ENDIF               
        
    ENDPROC
    
    PROC pick_up_brick_from_feed()
        
        ! sequence of poses for picking up a full brick
        VAR jointtarget home_jtarget;
        VAR robtarget home_rtarget;
        VAR robtarget brick_rtarget;
        
        ! poses and booleans for sensors
        VAR bool di_dist;
        VAR bool di_vac;
        VAR robtarget trigger_dist_pos;
        VAR robtarget trigger_vac_pos;
        
        home_jtarget:=current_jointpos; !home position for beginning and end of routine
        
        IF (act_tcpip_message_command.arbitrary = 0) THEN 
            brick_rtarget:=[[-668.36806, 133.49660, 610.56435] , [-0.03711, -0.70573, 0.70632, 0.04091],[0,0,0,0],[0,0,0,0,0,0]]; !pickup position of brick
        ELSE
            brick_rtarget:=[[-670.55,337.959,607.771],[0.04304,0.70954,-0.70248,-0.03490],[0,0,0,0],[0,0,0,0,0,0]]; !pickup position of half brick
        ENDIF
        
        di_dist:=FALSE;
        di_vac:=FALSE;
        
        SingArea \Off;
        ConfL \Off;
        PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;

        ! drive to home joint target
        MoveAbsJ home_jtarget,current_speed_data,current_zone_data,tool0;
        
        ! drive 80mm over over the given brickpose
        MoveL RelTool(brick_rtarget,0,0,-80),current_speed_data,current_zone_data,current_tool,\WObj:=wobj0;
        
        ! drive 10mm over over the given brickpose with current speed
        MoveL RelTool(brick_rtarget,0,0,-15),current_speed_data,current_zone_data,current_tool,\WObj:=wobj0;
        
        ! drive down until distance sensor kicks in            
        !SearchL \PStop,DI_DISTANCE,trigger_dist_pos,RelTool(brick_rtarget,0,0,10),v10,current_tool;   
        di_dist:=TRUE; 
            
        ! turn on vakuum pump
        !SetDO DO_VACPUMP,1;
        !SetDO DO_VALVE2,1;
        WaitTime 0.5;

        ! drive down until vacuum sensor kicks in 
        !SearchL \PStop,DI_VACUUM,trigger_vac_pos,RelTool(trigger_dist_pos,0,0,3),v5,current_tool; ! The robot is moved to a position that is 3 mm from pose hit_dist in the z direction of the tool.
        di_vac:=TRUE;
        
        MoveL RelTool(brick_rtarget,0,0,-160),v40,current_zone_data,current_tool,\WObj:=wobj0;
        
        home_rtarget:=CalcRobT(home_jtarget, current_tool);
        IF BUFFER_LENGTH > 1 THEN
                MoveLSync home_rtarget,current_speed_data,current_zone_data,current_tool,\WObj:=wobj0,"send_command_executed"; !MoveAbsJ home_jtarget,current_speed_data,fine,tool0; ! drive to home joint target
            ELSE
                MoveL home_rtarget,current_speed_data,fine,current_tool,\WObj:=wobj0; !MoveAbsJ home_jtarget,current_speed_data,fine,tool0; ! drive to home joint target
                send_command_executed;
        ENDIF
        
        ERROR
            IF ERRNO=ERR_WHLSEARCH THEN
                TPWrite "The signal was not triggered,dist/vac: " + ValToStr(di_dist) + " " + ValToStr(di_vac);
                MoveL RelTool(brick_rtarget,0,0,-100),v20,fine,current_tool,\WObj:=wobj0;
                Stop;
                RETURN;
 
            ELSEIF ERRNO=ERR_SIGSUPSEARCH THEN
                TPWrite "The signal of the SearchL instruction is already high!" + ValToStr(di_dist) + " " + ValToStr(di_vac);
                TRYNEXT;
            ENDIF
    ENDPROC
    
    PROC  place_brick_at_pose()
        ! place brick at robtarget with distance sensor
        
        ! poses and booleans for sensors
        VAR bool di_dist;
        VAR robtarget trigger_dist_pos;
        
        di_dist:=FALSE;
        
        SingArea \Off;
        ConfL \Off;
        PathAccLim TRUE\AccMax:=acc_max,TRUE\DecelMax:=dec_max;
        
        ! drive to brickpose + 100 in z
        MoveL RelTool(current_pos,0,0,-100),v20,current_zone_data,current_tool,\WObj:=wobj0;
        
        ! drive to brickpose + 50 in z
        MoveL RelTool(current_pos,0,0,-30),v20,current_zone_data,current_tool,\WObj:=wobj0;
        
        ! drive to brickpose -10 in z until distance sensor triggers
        !SearchL \PStop,DI_DISTANCE,trigger_dist_pos,RelTool(current_pos,0,0,10),v5,current_tool;   
        di_dist:=TRUE; 

        ! turn off vakuum pump
        !SetDO DO_VALVE2,0;
        !SetDO DO_VACPUMP,0;
        !WaitTime 0.1; 
        WaitTime 0.1;
        
        IF BUFFER_LENGTH > 1 THEN
                MoveL RelTool(current_pos,0,0,-20),v10,current_zone_data,current_tool,\WObj:=wobj0;
                MoveLSync RelTool(current_pos,0,0,-160),v50,current_zone_data,current_tool,\WObj:=wobj0,"send_command_executed";
            ELSE
                MoveL RelTool(current_pos,0,0,-20),v10,current_zone_data,current_tool,\WObj:=wobj0;
                MoveL RelTool(current_pos,0,0,-160),v50,fine,current_tool,\WObj:=wobj0;
                send_command_executed;
        ENDIF
        
        ERROR
            IF ERRNO=ERR_WHLSEARCH THEN
                TPWrite "The signal was not triggered,di_dist" + ValToStr(di_dist);
                MoveL RelTool(current_pos,0,0,-100),v20,fine,current_tool,\WObj:=wobj0;
                Stop;
                RETURN;

            ELSEIF ERRNO=ERR_SIGSUPSEARCH THEN
                TPWrite "The signal of the SearchL instruction is already high!,di_dist" + ValToStr(di_dist);
                Stop;
                TRYNEXT;
            ENDIF
        
    ENDPROC
    
    PROC send_command_executed()
        !after the movement is finished,send back the execute command
        send_msg_cmd_exec:=TRUE;
    ENDPROC
    
    FUNC bool calc_joint_reach(VAR robtarget current_p1)
        !calculate if the next point is reachable
        VAR bool j_reach:=TRUE;
        current_jointpos:=CalcJointT(current_p1,current_tool \WObj:=wobj0);
        RETURN j_reach;
        
        ERROR
        IF ERRNO = ERR_ROBLIMIT THEN
            SkipWarn;
            TPWrite "Joint current_p1 can not be reached.";
            j_reach:=FALSE;
            RETURN j_reach;
        ENDIF   
    ENDFUNC

    PROC go_to_init_pos()
        TPWrite "Move to init pos.";
        MoveL init_pos,mid,fine,current_tool,\WObj:=wobj0;
    ENDPROC
    
    PROC pick_up_brick_from_pose()
        VAR bool dist;
        VAR bool vacuum;
        
        VAR robtarget hit_dist;
        VAR robtarget hit_vac;
        
        !drive to brickpose + 50 in z
        current_pos.trans.z:=current_pos.trans.z + 100;
        MoveL current_pos,current_speed_data,current_zone_data,current_tool,\WObj:=wobj0;
        
        !drive down until distance sensor kicks in
        current_pos.trans.z:=current_pos.trans.z - 100;
        !SearchL \PStop,DI_DISTANCE,hit_dist,current_pos,v10,current_tool;
        
        hit_dist.trans.z:=hit_dist.trans.z - 18;
         
        !turn on vakuum pump
        !SetDO DO_VACPUMP,1;
        WaitTime 2;
        !SetDO DO_VALVE2,1;
        
        !drive down until vacuum sensor kicks in
        !SearchL \PStop,DI_VACUUM,hit_vac,hit_dist,v10,current_tool;
        
        current_pos.trans.z:=current_pos.trans.z + 200;
        
        IF BUFFER_LENGTH > 1 THEN
                MoveLSync current_pos,current_speed_data,current_zone_data,current_tool,\WObj:=wobj0,"send_command_executed";
            ELSE
                MoveL current_pos,current_speed_data,fine,current_tool,\WObj:=wobj0;
                send_command_executed;
        ENDIF     
    ENDPROC

ENDMODULE