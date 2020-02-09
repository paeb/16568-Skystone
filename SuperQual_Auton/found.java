       private Servo found1 = null;
        private Servo found2 = null;
       
       
       found1 = hardwareMap.get(Servo.class, "found1");
        found2 = hardwareMap.get(Servo.class, "found2");
        
        
        if (gamepad2.dpad_left){
            lastDPres = "left";
            found1.setPosition(1);
            found2.setPosition(.95);
        } else if (gamepad2.dpad_right){
            lastPres = "right";
            found1.setPosition(0);
            found2.setPosition(0);
        }
