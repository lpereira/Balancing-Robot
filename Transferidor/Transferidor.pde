import processing.serial.*;

PrintWriter outputFile = null;
Serial serialPort;
float angle = 0.0f;
PFont font;
PImage bg;

float zeroAngle = 90.0f;
float range = 180.0f;
float maxValue = 180.0f;

int outputFileNo = 0;

void setup() {
  size(498, 272);
  noStroke();

  println(Serial.list());

  String portName = Serial.list()[0];
  serialPort = new Serial(this, portName, 9600);
  serialPort.bufferUntil(10);

  font = loadFont("LucidaBright-DemiItalic-48.vlw");
  textFont(font, 30);
  fill(0, 102, 153);

  bg = loadImage("transferidor8115.jpg");

  strokeWeight(3);
  smooth();
}

void draw() {
  int x1 = 250;
  int x2 = x1; 
  int y1 = 244; 
  int y2 = y1;
  float angleToUse = zeroAngle - range * (angle / maxValue);

  x2 += (int)( cos(radians(angleToUse))*250);
  y2 -= (int)( sin(radians(angleToUse))*244);

  background(bg);
  stroke(255, 0, 0);
  line(x1, y1, x2, y2);
  ellipse(x1, y1, 5, 5);
  ellipse(x2, y2, 5, 5);

  text(angle, 260, 240);
  
  if (outputFile != null) {
    outputFile.println(angle + 90);
  }
}

void keyPressed() {
  if (key == RETURN || key == ENTER) {
      if (outputFile == null) {
          outputFileNo++;

          outputFile = createWriter(Integer.toString(outputFileNo) + ".txt");
          println("output file is: " + Integer.toString(outputFileNo) + ".txt");
      } else {
          println("closing output file: " + Integer.toString(outputFileNo) + ".txt");
          outputFile.flush();
          outputFile.close();
          outputFile = null;
      }
  }
}

void serialEvent(Serial serialPort) {
  String s = serialPort.readString();

  try {
    angle = Float.parseFloat(s);
  } 
  catch (java.lang.NumberFormatException e) {
    angle = 0;
  }

  angle = -angle;
}


