MODULE Motion

    VAR robtarget home_pos;
    
        
    PROC main()
        
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