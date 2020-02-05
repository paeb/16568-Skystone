    private int flPosition;
    private int frPosition;
    private int blPosition;
    private int brPosition;
    private double strafeAngle;
    private Orientation angles;
    private int acceptableRange = 3;
    private double increment = -.25;
    private double power = .65;
    
    public double compareAngles(AngleUnit angleUnit, double angle) {
        double degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);

        return AngleUnit.DEGREES.normalize(degrees);
    }
    
    private void strafe(int distance){
        strafeAngle = compareAngles(angles.angleUnit, angles.firstAngle);
        double circumTraveled = distance / wheelCircum;
        int position = (int) (ticksPerTurn * circumTraveled);

        if (position < 0) {
            position = -position; //go backwards
        }
        else if (position > 0) {
            position = position; //go forwards
        }
        else {
            position = 0;
        }

        flPosition = fl.getCurrentPosition() - position;
        frPosition = fr.getCurrentPosition() + position;
        blPosition = bl.getCurrentPosition() + position;
        brPosition = br.getCurrentPosition() - position;

        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    
    
    
            case -3:
                flReached = Math.abs(fl.getCurrentPosition()) >= Math.abs(targetPosition) - 20;
                frReached = Math.abs(fr.getCurrentPosition()) >= Math.abs(targetPosition) - 20;
                blReached = Math.abs(bl.getCurrentPosition()) >= Math.abs(targetPosition) - 20;
                brReached = Math.abs(br.getCurrentPosition()) >= Math.abs(targetPosition) - 20;

                if (flReached && frReached && blReached && brReached){
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);
                } else {
                    double v1correction = 0;
                    double v2correction = 0;
                    double v3correction = 0;
                    double v4correction = 0;
                    if (compareAngles(angles.angleUnit, angles.firstAngle) - strafeAngle < -acceptableRange){
                        v1correction = -increment;
                        v2correction = increment;
                        v3correction= -increment;
                        v4correction = increment;
                    } else if (compareAngles(angles.angleUnit, angles.firstAngle) - strafeAngle > acceptableRange) {
                        v1correction = increment;
                        v2correction = -increment;
                        v3correction = increment;
                        v4correction = -increment;
                    }
                    fl.setPower(power+v1correction);
                    fr.setPower(power+v2correction);
                    bl.setPower(power+v3correction);
                    br.setPower(power+v4correction);
                }

                break;
