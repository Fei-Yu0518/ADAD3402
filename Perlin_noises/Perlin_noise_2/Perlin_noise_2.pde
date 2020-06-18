float noiseScale = 0.01;
void setup() {
  size(700, 500);
}
void draw() {
  background(0);
  for (int x = 0; x < width; x++){
    for(int y = 0; y < height; y++){
      float noiseVal = noise(x*noiseScale, y*noiseScale);
      stroke(noiseVal*255,map(mouseX, 0, width, 0, 150), map(mouseY, 0, height, 150, 255));
      point(x, y);
    }
  }
}
