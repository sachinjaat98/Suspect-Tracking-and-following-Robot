
// defines pins numbers
const int trigPin = 10;
const int echoPin = 11;

//int sig = 11;
// defines variables
long duration;
int distance;

const int lmf = 3;
const int lmb = 5;
const int rmf = 6;
const int rmb = 9;


void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
 

}

void loop()
{
  delay(50);
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration / 58.2;

  //int val = digitalRead(sig);

  if (distance < 15)
  {
    stopit();
    left();
    Serial.println("left");
    delay(200);
  }

  else
  {
    forward();
    Serial.println("forward");
  }



}

void forward()
{
  //analogWrite(LM_speed, 155);
  analogWrite(lmf, 128);
  analogWrite(lmb, 0);
  analogWrite(rmf, 150);
  analogWrite(rmb, 0);
}

void reverse()
{
  //analogWrite(LM_speed, 255);
  analogWrite(lmf, 0);
  analogWrite(lmb, 128);
  analogWrite(rmf, 0);
  analogWrite(rmb, 150);
}
void stopit()
{

  analogWrite(lmf, 0);
  analogWrite(lmb, 0);
  analogWrite(rmf, 0);
  analogWrite(rmb, 0);
}
void left()
{

  analogWrite(lmf, 0);
  analogWrite(lmb, 200);
  analogWrite(rmf, 200);
  analogWrite(rmb, 0);
}
