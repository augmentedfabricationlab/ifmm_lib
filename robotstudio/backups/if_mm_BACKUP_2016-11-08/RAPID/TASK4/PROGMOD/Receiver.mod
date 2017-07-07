MODULE Receiver
    PROC main()
        VAR TCPIP_MSG_COMMAND act_message_command;
        VAR TCPIP_MSG_RAW_RECEIVER act_message_raw;
        VAR num msg_type;    
        
        init; !Set all variables

        TPWrite "Receiver Server: Waiting for connection.";
	    init_socket socket_server,port_receiver; !create server and wait for incoming connections
        wait_for_client socket_server,socket_client \wait_time:=1.0; !wait_for_client socket_server,socket_client;
        
        WHILE ( TRUE ) DO
                     
            IF BUFFER_LENGTH < MAX_BUFFER_SIZE THEN
                
                ! receive the message from the socket
                TCPIP_receive_msg socket_client, act_message_raw;
                
                ! set the message received counter and send back "message received".
                msg_rec_counter:=act_message_raw.msg_counter;
                send_msg_cmd_rcv:=TRUE;
                
                IF act_message_raw.msg_type = MSG_COMMAND THEN
                    TCPIP_receive_msg_command act_message_raw, act_message_command;
                    WaitTestAndSet msg_lock;  !acquire data-lock
                        tcpip_message_command_buffer{WRITE_PTR}:=act_message_command;
                        BUFFER_LENGTH:=BUFFER_LENGTH + 1;
                        WRITE_PTR:=(WRITE_PTR MOD MAX_BUFFER_SIZE) + 1;
                    msg_lock:=FALSE;
                
                ELSEIF act_message_raw.msg_type = MSG_STOP THEN
                    TPWrite "Message from Server: STOP";
                ELSEIF act_message_raw.msg_type = MSG_IDLE THEN
                    TPWrite "Message from Server: IDLE";  
                ELSE
                    ErrWrite \W,"SANS Socket Message Id Mismatch","Unexpected message id",
                        \RL2:="received: " + ValToStr(act_message_raw.msg_type);
                    RAISE ERR_ARGVALERR;  !KD: define specific error code
                ENDIF
            ENDIF
        ENDWHILE

        ERROR (ERR_SOCK_CLOSED)
        	IF (ERRNO=ERR_SOCK_CLOSED) THEN
                SkipWarn;  !TBD: include this error data in the message logged below?
                ErrWrite \W,"SANS Receiver disconnect","Connection lost.  Waiting for new connection.";
                
                reset_after_connection_stop;
                ExitCycle;  !restart program
                
        	ELSE
        		TRYNEXT;
        	ENDIF
        UNDO
        
    ENDPROC
    
    
ENDMODULE