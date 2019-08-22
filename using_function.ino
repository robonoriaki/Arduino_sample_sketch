

void setup() {
  Serial.begin(9600);
}

void loop() {
  int a = 5;
  int b = 2;
  int x;

  x = add(a,b);

  Serial.print("x = ");
  Serial.print(x);
  Serial.print("\n");

  return 0;
}

int add(int i, int j){
  int k;

  k = i + j;

  return k;
}

