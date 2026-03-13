MODULE PLC_Data
    PERS bool gPerfEnable := TRUE;
    
	PERS num x1_off := -35;
    PERS num y1_off := 0;
    PERS num z1_off := 125;
    
    VAR robtarget rel_move;
    VAR num cut_angle;
    VAR num speed_override;
    VAR intnum gi_value;
    VAR egmident egmId;
    VAR num defaultStateChangeTimeout := 10; ! seconds
    VAR num HomeSequenceTimeout := 45; ! seconds
    VAR bool trapConnected := FALSE;
    
	! Timer persistent values
    PERS num nCycleCount := 10;
    PERS num nLastPly := 0; 
    PERS string logFilePath := "/process_times.txt";
	
	! Timer variable values
    VAR clock clkCourse;    ! Total time from Temp_Offset to end of Cut
    VAR clock clkPly;       ! Total time for the entire Ply
    VAR iodev logFile;
	
    PROC LogToFile(string msg)
        VAR iodev logFile;
        Open "HOME:" + logFilePath, logFile \Append;
        Write logFile, CDate() + " " + CTime() + " | " + msg;
        Close logFile;
    ERROR
        IF ERRNO = ERR_FILEACC THEN
            TPWrite "LOG ERROR: File access failed.";
            TRYNEXT;
        ENDIF
    ENDPROC
    
    PROC OpsStartPLC()        
        VAR E_RobotState eRobotState;
        VAR num robotState;
        VAR num currentPlcState;
		
        !EGM_Setup;
        Reset Rbt_PreAlignPos;
        Reset Rbt_Home;
        Reset out_CourseFin;
        Reset Rbt_KnifeServiceReady;
        eRobotState := defineRobotStates();
        currentPlcState := RequestedPlcState;
        IF NOT trapConnected THEN
            CONNECT gi_value WITH gi_change;
            ISignalGI GI_Speed, gi_value;
            trapConnected := TRUE;
            WaitTime 1;
        ENDIF
        IF PlcState = 255 or PlcState = 0 THEN
            robotState := ChangeToState(eRobotState.StartUp, defaultStateChangeTimeout);
            robotState := ChangeToState(eRobotState.Idle, HomeSequenceTimeout);
        ELSEIF PlcState = 1 THEN
            robotState := ChangeToState(eRobotState.Idle, HomeSequenceTimeout);
        ENDIF
    ENDPROC
    
    
    PROC Temp_Offset(robtarget Offset_name, PERS wobjdata work_objrep)
		VAR clock cLocal;
        VAR num dt;
        VAR string logHeader;

            ! Ply timing logic
            IF RequestedPly <> nLastPly THEN
                ClkReset clkPly;
                ClkStart clkPly;
                nLastPly := RequestedPly;
                LogToFile ">>>> STARTING PLY " + NumToStr(RequestedPly, 0);
            ENDIF

		logHeader := "Ply:" + NumToStr(RequestedPly, 0) + " C:" + NumToStr(RequestedCourse, 0);
        MoveL RelTool(Offset_name, x1_off, y1_off, z1_off), v500, fine, RTSHeadTool\WObj:=work_objrep;
		
        IF Dry_Run = LOW THEN
            SetDO Rbt_PreAlignPos, HIGH;
    
			IF gPerfEnable THEN
				! Course timer
				ClkReset clkCourse;
				ClkStart clkCourse;
				
				! Local timer
				ClkReset cLocal;
				ClkStart cLocal;
			ENDIF
    
            WHILE TRUE DO
                ! Wait for either success OR retry request
                WaitUntil (PLC_AlignComp = HIGH) OR (Rbt_RetryPreAlign = HIGH);
                ! Success
                IF PLC_AlignComp = HIGH THEN
                    Reset Rbt_PreAlignPos;
                    GOTO ready;
                ENDIF
    
                ! Retry requested:
                Reset Rbt_PreAlignPos;
                rel_move := RelTool(CRobT(), -10, 0, 0);
                MoveL rel_move, v100, fine, RTSHeadTool\WObj:=work_objrep;
    
                SetDO Rbt_PreAlignPos, HIGH;
    
                ! Wait for PLC to acknowledge and clear retry request
                WaitUntil Rbt_RetryPreAlign = LOW;
            ENDWHILE
        ENDIF
    ready:
    
		IF gPerfEnable THEN
            ClkStop cLocal;
            dt := ClkRead(cLocal);
            LogToFile logHeader + " | Pre-pos: " + NumToStr(dt, 2) + "s";
        ENDIF
    ENDPROC


    PROC LayPrep()
        VAR E_RobotState eRobotState;
        VAR num robotState;
        eRobotState := defineRobotStates(); 
        IF Dry_Run = LOW THEN
            Reset Rbt_CutReady;
            Reset Rbt_ConsReady;
            robotState := ChangeToState(eRobotState.ReadyToLay, defaultStateChangeTimeout);
            robotState := ChangeToState(eRobotState.Layup, defaultStateChangeTimeout);
            PathAccLim TRUE\AccMax:=0.45, TRUE\DecelMax:=1;
        ENDIF
    ENDPROC
    
    PROC CutProcess(robtarget Offset_name, num cut_angle, PERS wobjdata work_objrep)
		VAR clock cCut; ! Local timer for the cut movement
        VAR num dtCut;
        VAR num dtTotalCourse;
        VAR string logBase;
		
		VAR E_RobotState eRobotState;
        VAR num robotState;
		eRobotState := defineRobotStates();
		
		logBase := "Ply:" + NumToStr(RequestedPly, 0) + " C:" + NumToStr(RequestedCourse, 0);
		
        IF Dry_Run = HIGH THEN			
            SETDO out_CourseFin, HIGH;
            MoveL RelTool(Offset_name, -53, 0, 0),v100,fine,RTSHeadTool\WObj:=work_objrep;       !-53 in x
            MoveL RelTool(Offset_name, -53, 0, -53),v60,fine,RTSHeadTool\WObj:=work_objrep;     !-53 in z
            MoveL RelTool(Offset_name, -53, 0, -208),v300,fine,RTSHeadTool\WObj:=work_objrep;    !-155 in z
            MoveL RelTool(Offset_name, -263, 0, -208),v300,z10,RTSHeadTool\WObj:=work_objrep;   !-210 in x
            MoveL RelTool(Offset_name, -263, 0, -308),v500,z10,RTSHeadTool\WObj:=work_objrep;   !-100 in z
            Reset out_CourseFin;
            MoveL RelTool(Offset_name, -63, 0, -308),v400,z80,RTSHeadTool\WObj:=work_objrep;    !200 in x
        ELSE
			nCycleCount := nCycleCount + 1;

            ! Start timer for the cut movement specifically
            IF gPerfEnable THEN
                ClkReset cCut;
                ClkStart cCut;
            ENDIF
			
            SetGO RequestedCutAngle, cut_angle;
            robotState := ChangeToState(eRobotState.CutReady, defaultStateChangeTimeout);
            WaitTime 0.6;
            MoveL RelTool(Offset_name, -63, 0, 0),v200,fine,RTSHeadTool\WObj:=work_objrep;       !-53 in x
            SETDO out_CourseFin, HIGH;
            PathAccLim FALSE, FALSE;
            MoveL RelTool(Offset_name, -63, 0, -53),v100,fine,RTSHeadTool\WObj:=work_objrep;     !-53 in z
            ! IO for consolidation control
            Set Rbt_ConsReady;
            WaitDI Rbt_ConsPosDone,1;
            MoveL RelTool(Offset_name, -63, 0, -200),v500,fine,RTSHeadTool\WObj:=work_objrep;    !-140 in z (Z was -174)
            robotState := ChangeToState(eRobotState.Cut, defaultStateChangeTimeout);
            ! IO for cut control
            Set Rbt_CutReady;
            WaitDI in_CutComplete, 1;
            PathAccLim TRUE\AccMax:=0.5, TRUE\DecelMax:=0.2;
            MoveL RelTool(Offset_name, -263, 0, -200),v500,z10,RTSHeadTool\WObj:=work_objrep;   !-210 in x 150 for low cut
            MoveL RelTool(Offset_name, -263, 0, -265),v600,z10,RTSHeadTool\WObj:=work_objrep;   !-100 in z (was -274)
            robotState := ChangeToState(eRobotState.FinishCourse, defaultStateChangeTimeout);
            robotState := ChangeToState(eRobotState.Idle, defaultStateChangeTimeout);
            Reset out_CourseFin;
            MoveL RelTool(Offset_name, -63, 0, -265),v600,z50,RTSHeadTool\WObj:=work_objrep;    !200 in x (was -274)
        ENDIF
        
			IF gPerfEnable THEN
                ! Timers stopped
                ClkStop cCut;
                ClkStop clkCourse;
                
                dtCut := ClkRead(cCut);
                dtTotalCourse := ClkRead(clkCourse);
                
                ! Log detailed breakdown
                LogToFile logBase + " | Cut Time: " + NumToStr(dtCut, 2) + "s";
                LogToFile logBase + " | TOTAL COURSE TIME: " + NumToStr(dtTotalCourse, 2) + "s";
            ENDIF
    ENDPROC
    
    PROC main()     
        OpsStartPLC;
        IF LoadPreformProgram = 1 THEN
            SetDO PreformProgramLoaded, low;
            LoadProgram "PreformProgTemplateV1";
        ELSEIF KnifeServiceRequest = 1 THEN
            KnifeService;
        ElSEIF Load_Lift = 1 THEN
            LiftHead;
            WaitUntil out_NOTRbtTable = high;
        ELSEIF binto_home = 1 THEN
           into_home;
           WaitUntil out_NOTRbtHome = low;
        ELSEIF bRunProgram = 1 THEN
            IF in_CBCPerform = 1 THEN
                CyclicBrakeCheck;
            ELSEIF out_NOTRbtHome = 0 THEN
                out_of_home;
                WaitUntil out_NOTRbtHome = high;
            ELSE
                ! DO NOT DELETE
                SelectCourse_Ply(RequestedPly), (RequestedCourse);
            ENDIF
        ENDIF
    ENDPROC


    PROC LoadProgram(string filename)
        VAR string filepath;
        filepath := "/hd0a/7600-107413/HOME/" + filename + ".mod";
    
        SetDO PreformProgramLoaded, 0;
    
        IF ModExist(filename) THEN
            EraseModule filename;
        ENDIF
    
        Load filepath \CheckRef;
    
        IF ModExist(filename) THEN
            SetDO PreformProgramLoaded, 1;
        ELSE
            TPWrite "LoadProgram: module missing after load";
            SetDO PreformProgramLoaded, 0;
            RETURN;
        ENDIF
    
        RETURN;
    
    ERROR
        TPWrite "LoadProgram failed, ERRNO=" \Num:=ERRNO;
        SetDO PreformProgramLoaded, 0;
        RAISE;
    ENDPROC
    
    
    !_________ Trap routine ___________________
    TRAP gi_change
        gi_value := GI_Speed;
        speed_override := gi_value;

        IF speed_override > 100 THEN
            speed_override := 100;
        ENDIF
        IF speed_override < 0 THEN
            speed_override := 0;
        ENDIF
        SpeedRefresh speed_override;
    ENDTRAP
 
ENDMODULE