/*
  This example shows how to use Image, Array and Texture to read a .jpg file,
display it as an OpenGL texture and print the pixel values on the command line.
Notice that while the intput image has only 4 pixels, the rendered texture is
smooth.  This is because interpolation is done on the GPU.
  Karl Yerkes and Matt Wright (2011/10/10)
*/

#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "al/app/al_App.hpp"
#include "al/graphics/al_Image.hpp"

using namespace al;
using namespace std;

class MyApp : public App {
 public:     
  Mesh pic, rgb, hsv, somthing_else;
  Mesh actual;
  int W,H;

  void onCreate() override {
    const char *filename = "data/icon.jpg";
    
    auto imageData = Image(filename);
    if (imageData.array().size() == 0) {
      cout << "failed to load image" << endl;
      exit(1);
    }
    cout << "loaded image size: " << imageData.width() << ", "
         << imageData.height() << endl;

    if(imageData.width()>1000||imageData.height()>1000){
      cout<<"The loaded image is bigger than 1000x1000.";
      return;
    }

    W = imageData.width();
    H = imageData.height();
    pic.primitive(Mesh::POINTS);
    actual.primitive(Mesh::POINTS);

    // iterate through all the pixel, scanning each row
    for (int row = 0; row < H; row++) {
      for (int column = 0; column < W; column++) {
        auto pixel = imageData.at(column, H - row - 1);
        pic.vertex(1.0 * column / W, 1.0 * row / H, 0.0);
        pic.color(pixel.r / 255.0, pixel.g / 255.0, pixel.b / 255.0);
        
        // initial setting of actual
        actual.vertex(1.0 * column / W, 1.0 * row / H, 0.0);
        actual.color(pixel.r / 255.0, pixel.g / 255.0, pixel.b / 255.0);
        
      }
    }

    // set the camera position back some (z=3) and center on (x=0.5, y=0.5)
    nav().pos(0.5, 0.5, 3);

  }

  void onAnimate(double dt) override {
    // hint modify `actual`
    // make an animation parameter (float) `t`
  }

  bool onKeyDown(const Keyboard & k) override {
    // Use a switch to do something when a particular key is pressed
    switch (k.key()) {
      // For printable keys, we just use its character symbol:
      case '1':
        for (int row = 0; row < H; row++) {
          for (int column = 0; column < W; column++) {
            this->pic.colors()[row+column*W].set(this->actual.colors()[row+column*W],1);            
            this->pic.vertices()[row+column*W].set(this->actual.vertices()[row+column*W],1);
          }
        }
        break;
      case '2':
        cout<<"Placing the points of the image in RGB Cube space..."<<endl;
        for (int row = 0; row < H; row++) {
          for (int column = 0; column < W; column++) {
            this->pic.colors()[row+column*W].set(this->actual.colors()[row+column*W],1);
          }
        }
        break;
      case '3':      
         cout<<"Placing the points of the image in HSV Cylinder space..."<<endl;
        cout<< HSV(this->pic.colors()[4000]).h<<endl;
        for (int row = 0; row < H; row++) {
          for (int column = 0; column < W; column++) {
            this->pic.colors()[row+column*W].set(HSV(this->pic.colors()[row+column*W]),1); 
          }
        }
        //onAnimate(1);
        break;
      case '4':
        for (int row = 0; row < H; row++) {
          for (int column = 0; column < W; column++) {
            this->pic.colors()[row+column*W].set(0,255,0); 
          }
        }
        break;
    
        // hint reset animation parameter

      default:
        break;
    }
    return true;
  }

  void onDraw(Graphics &g) override {
    g.clear(0.2f);
    g.meshColor();
    g.draw(pic);
  }
};

int main() {
  MyApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}