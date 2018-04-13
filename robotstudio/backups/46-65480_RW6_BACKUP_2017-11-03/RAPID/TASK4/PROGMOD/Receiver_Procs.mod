MODULE Receiver_Procs
    
    !for client
    VAR num retry_no:=0;
    
    VAR socketdev receiver_socket;
    
    VAR socketdev socket_server;
    VAR socketdev socket_client;

    
    PROC init()
        
        !initialize robot
        TPWrite "Initialization Receiver Client";
        
        msg_lock:=FALSE;
        
        msg_rec_counter:=0;
        
        BUFFER_LENGTH:=0;
        READ_PTR:=1;
        WRITE_PTR:=1;
        
        TPWrite "Write pointer:" + ValToStr(WRITE_PTR);
       
        msg_send_counter_lock:=FALSE;
    
    ENDPROC
    
    PROC reset_after_connection_stop()
  
        TPWrite "CONNECTION ERROR, pp will be moved to main, vars are reset";
        
        msg_lock:=FALSE;
        msg_send_counter_lock:=FALSE;
        
        msg_rec_counter:=0;
        cmd_exec_counter:=0;
        
        BUFFER_LENGTH:=0;
        READ_PTR:=1;
        WRITE_PTR:=1;
        
        reset_to_main:=TRUE;
        
        !StopMove;
        !ClearPath;
        !StopMoveReset;
        !StartMove;
        
    ENDPROC
    
    
    
ENDMODULE