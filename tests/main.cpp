#include<tiny-3D-loader.hpp>

int main()
{
  tiny3Dloader::glTFLoader loader;
  loader.load("models/Lantern.gltf", "models/");
  return 0;
}
