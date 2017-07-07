MODULE Motion

    VAR robtarget home_pos;
    
    VAR robtarget weight_test_pos;
    PERS speeddata testspeed:=[300,30,0,0];
  
    PROC main()
        
        !current_speed_data := slow;
        !weight_test_pos:=CRobT(\Tool:=weighttest \WObj:=wobj0);
        !FOR i FROM 1 TO 10 DO
        !    TPWrite "Hello drive loopy";
        !    MoveL RelTool(weight_test_pos,500,0,0),testspeed,fine,weighttest,\WObj:=wobj0;
        !    !MoveL RelTool(weight_test_pos,50,0,0,\Rx:=10),testspeed,current_zone_data,tool0,\WObj:=wobj0;
        !    weight_test_pos:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        !    !WaitTime 0.1;  
        !    MoveL RelTool(weight_test_pos,0,200,0),testspeed,fine,weighttest,\WObj:=wobj0;
        !    weight_test_pos:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        !    !WaitTime 0.1;
        !    MoveL RelTool(weight_test_pos,-500,0,0),testspeed,fine,weighttest,\WObj:=wobj0;
        !    weight_test_pos:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        !    !WaitTime 0.1;
        !    MoveL RelTool(weight_test_pos,0,-200,0),testspeed,fine,weighttest,\WObj:=wobj0;
        !    weight_test_pos:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        !    !WaitTime 0.1;
        !ENDFOR

        
        init;
        TPWrite "Motion: Start waiting for positions in buffer and move accordingly";
        
        WHILE ( TRUE ) DO  
            IF BUFFER_LENGTH > 0 THEN
                IF msg_lock = FALSE THEN
                    execute_from_buffer;
                ENDIF
            ENDIF
            WaitTime 0.05;
        ENDWHILE  
        
    ENDPROC
    
ENDMODULE
