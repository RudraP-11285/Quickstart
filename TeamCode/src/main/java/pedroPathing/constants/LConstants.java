package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0029778970344696482; //0.001989436789;
        ThreeWheelConstants.strafeTicksToInches = 0.0029553260086358423; //0.001989436789;
        ThreeWheelConstants.turnTicksToInches = -0.0030278507807813162; //.001989436789;
        ThreeWheelConstants.leftY = 5.75;
        ThreeWheelConstants.rightY = -5.75;
        ThreeWheelConstants.strafeX = 6.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "rightBack";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




