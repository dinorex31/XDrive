#include "main.h"

class PolarPoint{
  public:
    PolarPoint(double r, double theta){
      if(r < 0){
        radius = -r;
        angle = std::fmod((theta + M_PI) , (2 * M_PI));
      }else{
        radius = r;
        angle = theta;
      };
    };
    void divideBy(double num){
      if(num == 0){
        radius = 0;
      }else{
        radius = radius / num;
      }
    };
    double getRadius(){
      return radius;
    };
    void limitRadius(){
      if(radius > 1){
        radius = 1;
      }
    }
    void rotateAngle(double theta){
      angle += theta;
    }
    double getAngle(){
      return angle;
    }
  private:
    double radius;
    double angle;
};

class Point{
  public:
    Point(double x, double y){
      this->x = x;
      this->y = y;
    };
    Point add(Point other){
      return Point(x + other.x, y + other.y);
    };
    void addLocally(Point other){
      x += other.x;
      y += other.y;
    }
    double getX(){
      return x;
    };
    double getY(){
      return y;
    };
  private:
    double x;
    double y;
};

Point toCartesian(PolarPoint p){
  return Point(p.getRadius() * std::cos(p.getAngle()), p.getRadius() * std::sin(p.getAngle()));
};

double toRadians(double degrees){
  return M_PI * degrees / 180;
};

double toDegrees(double radians){
  return 180 * radians / M_PI;
};

double unit(double num){
  if(num >= 0){
    return 1;
  }else{
    return -1;
  }
};

double largest(double arr[], int len){
  double val = -1000000000;
  for(int i = 0; i < len; i++){\
    if(arr[i] > val){
      val = arr[i];
    }
  }
  return val;
}

double largestMagnitude(double arr[], int len){
  double val = 0;
  for(int i = 0; i < len; i++){
    double positive = std::abs(arr[i]);
    if(positive > val){
      val = positive;
    }
  }
  return val;
}

double safeDivide(double a, double b){
  if (b == 0){
    return 0;
  }else{
    return a/b;
  }
}

double roundOutput(double num){
  double epsilon = 0.00000001;
  if((num - epsilon <= 0) && (num + epsilon >= 0)){
    return 0;
  }else{
    return num;
  }
}

double map(double x, double pmin, double pmax, double min, double max){
  return (x - pmin)*(max - min)/(pmax - pmin)+min;
}

int convertToVoltage(double t){
  return std::floor(map(t, -1, 1, -127, 127));
}

double convertToPercent(int t){
  return map(t, -127, 127, -1, 1);
}

void print(Point p){
  std::cout << "(" << p.getX() << ", " << p.getY() << ")\n";
}

void print(PolarPoint p){
  std::cout << "(" << p.getRadius() << ", " << toDegrees(p.getAngle()) << ")\n";
}

//field centric should simply need the current angle subtracted
double getWheelOutput(PolarPoint stick, double angleOffset){
  double radians = std::fmod((stick.getAngle() + angleOffset + 2 * M_PI - M_PI_4), (2 * M_PI));
  return stick.getRadius() * std::sin(radians);
}

Point getWheelVector(double speed, double rotation){
  return Point(std::cos(rotation) * speed, std::sin(rotation) * speed);
}

PolarPoint toPolar(Point other){
	double x = other.getX();
	double y = other.getY();
	if(x == 0){
		if(y > 0){
			return PolarPoint(y, M_PI_2);
		}else if(y < 0){
			return PolarPoint(std::abs(y), -M_PI_2);
		}else{
			return PolarPoint(0, 0);
		}
	}else{
		return PolarPoint(std::sqrt(x * x + y * y), std::atan2(y, x));
	}
};


double pLeftEnc = 0;
double pRightEnc = 0;
double pBackEnc = 0;
double pTheta = 0;
double leftEncAtLastReset = 0;
double rightEncAtLastReset = 0;
double thetaAtLastReset = 0;
const double lRadius = 7.25;
const double rRadius = 7.25;
const double bRadius = 7.75;
Point globalPosition = Point(0,0);

void updatePosition(double leftEnc, double rightEnc, double backEnc){
  double dLeftEnc = leftEnc - pLeftEnc;
  double dRightEnc = rightEnc - pRightEnc;
  double dBackEnc = backEnc - pBackEnc;


  double theta = thetaAtLastReset + safeDivide((leftEnc - leftEncAtLastReset) - (rightEnc - rightEncAtLastReset), lRadius + rRadius);

  double dTheta = theta - pTheta;

  double centerYArcRadius = safeDivide(dRightEnc, dTheta) + rRadius;

  double centerXArcRadius = safeDivide(dBackEnc,dTheta) + bRadius;


  //local coordinates
  double dX;
  double dY;
  if(dTheta == 0){
    dX = backEnc;
    dY = dRightEnc;
  }else{
    dX = 2 * std::sin(dTheta/2) * centerXArcRadius;
    dY = 2 * std::sin(dTheta/2) * centerYArcRadius;
  }

  PolarPoint coordinate = toPolar(Point(dX, dY));//local
  // print(toCartesian(coordinate));
  coordinate.rotateAngle(-(pTheta + dTheta/2));//global
  Point cartesianCoordinate = toCartesian(coordinate);
  // std::cout << "d: \n";
  // print(cartesianCoordinate);

  globalPosition.addLocally(cartesianCoordinate);

  pLeftEnc = leftEnc;
  pRightEnc = rightEnc;
  pBackEnc = backEnc;
  pTheta += dTheta;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 bool debug = true;

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor leftf_mtr(1);
	pros::Motor leftb_mtr(2);
	pros::Motor rightf_mtr(3);
	pros::Motor rightb_mtr(4);
	while (true) {
		Point joystickInput = Point(convertToPercent(master.get_analog(ANALOG_RIGHT_X)), convertToPercent(master.get_analog(ANALOG_RIGHT_Y)));
		PolarPoint translationInput = toPolar(joystickInput);
		translationInput.limitRadius();
		// std::cout << "translation input (polar): ";
		// print(translationInput);

		double rotInput = convertToPercent(master.get_analog(ANALOG_LEFT_X));
    pros::lcd::set_text(1, std::to_string(rotInput));
    // std::cout << "rotation input: " << rotInput << "\n";

		PolarPoint l1Turn = PolarPoint(rotInput, toRadians(45));
		PolarPoint l2Turn = PolarPoint(rotInput, toRadians(135));
		PolarPoint r1Turn = PolarPoint(rotInput, toRadians(135));
		PolarPoint r2Turn = PolarPoint(rotInput, toRadians(45));
    if(master.get_digital(DIGITAL_X) && debug){
      print(l1Turn);
      debug = false;
    }else if(!master.get_digital(DIGITAL_X)){
      debug = true;
    }
		double l1Out = getWheelOutput(translationInput, M_PI_2);
		double l2Out = getWheelOutput(translationInput, 0);
		double r1Out = -getWheelOutput(translationInput, 0);
		double r2Out = -getWheelOutput(translationInput, M_PI_2);


		double l1Rot = getWheelOutput(l1Turn, M_PI_2);
		double l2Rot = getWheelOutput(l2Turn, 0);
		double r1Rot = getWheelOutput(r1Turn, 0);
		double r2Rot = getWheelOutput(r2Turn, M_PI_2);

		double outArr [] = {l1Out, l2Out, r1Out, r2Out};
		double largestOut = largest(outArr, sizeof(outArr)/8);
		for(int i = 0; i < sizeof(outArr)/8; i++){
			outArr[i] = safeDivide(outArr[i], largestOut);
		}

		double rotArr [] = {l1Rot, l2Rot, r1Rot, r2Rot};
		double largestRot = largest(rotArr, sizeof(rotArr)/8);
		for(int i = 0; i < sizeof(rotArr)/8; i++){
      //TODO figure out permanent fix
			rotArr[i] = safeDivide(rotArr[i], rotInput < 0 ? -largestRot : largestRot);
		}

		// std::cout << "translation output: " << l1Out << ", "<< l2Out << ", " << r1Out << ", " << r2Out << "\n";
    //
		// std::cout << "rotation output: " << rotArr[0] << ", "<< rotArr[1] << ", " << rotArr[2] << ", " << rotArr[3] << "\n";
    //
		// std::cout << "total (normalized) output: " << outArr[0] << ", "<< outArr[1] << ", " << outArr[2] << ", " << outArr[3] << "\n";

		double totalSum = std::abs(rotInput) + std::abs(translationInput.getRadius());
		double rotPriority = rotInput/totalSum;
		double transPriority = translationInput.getRadius()/totalSum;

		for(int i = 0; i < sizeof(outArr)/8; i++){
			outArr[i] = outArr[i] * transPriority * translationInput.getRadius();
		}

		for(int i = 0; i < sizeof(rotArr)/8; i++){
			rotArr[i] = rotArr[i] * rotPriority * rotInput;
		}

		double wheelOutputs [] = {
			outArr[0] + rotArr[0],
			outArr[1] + rotArr[1],
			outArr[2] + rotArr[2],
			outArr[3] + rotArr[3]
		};

		// std::cout << "wheel outputs: " << wheelOutputs[0] << ", "<< wheelOutputs[1] << ", " << wheelOutputs[2] << ", " << wheelOutputs[3] << "\n";
		//
		// Point vecArr [4] = {
		// 	getWheelVector(wheelOutputs[0], M_PI_4),
		// 	getWheelVector(wheelOutputs[1], 3*M_PI_4),
		// 	getWheelVector(wheelOutputs[2], 3*M_PI_4),
		// 	getWheelVector(wheelOutputs[3], M_PI_4)
		// };
		//
		// PolarPoint polarVecArr [4] = {
		// 	toPolar(vecArr[0]),
		// 	toPolar(vecArr[1]),
		// 	toPolar(vecArr[2]),
		// 	toPolar(vecArr[3])
		// };
		//
		// polarVecArr[2].divideBy(-1);
		// polarVecArr[3].divideBy(-1);
		// vecArr[2] = toCartesian(polarVecArr[2]);
		// vecArr[3] = toCartesian(polarVecArr[3]);
		//
		// std::cout << "polar outputs: \n";
		// for(int i = 0; i < 4; i++){
		// 	print(polarVecArr[i]);
		// }
		//
		// std::cout << "cartesian outputs: \n";
		// for(int i = 0; i < 4; i++){
		// 	print(vecArr[i]);
		// }
		//
		// Point vecSum = Point(0,0);
		//
		// for(int i = 0; i < 4; i++){
		// 	vecSum = vecSum.add(vecArr[i]);
		// }
		//
		// std::cout << "total translation output (cartesian): ";
		// print(vecSum);
		// std::cout << "total translation output (polar): ";
		// print(toPolar(vecSum));
    leftf_mtr.move(convertToVoltage(wheelOutputs[0]));
    leftb_mtr.move(convertToVoltage(wheelOutputs[1]));
    rightf_mtr.move(convertToVoltage(wheelOutputs[2]));
    rightb_mtr.move(convertToVoltage(wheelOutputs[3]));
		pros::delay(20);
	}
}
