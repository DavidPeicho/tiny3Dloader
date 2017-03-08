#include <tiny-3D-loader.hpp>
#include <gtest/gtest.h>

TEST(MissingAsset, MissingBinaryFile) {

  std::vector<tiny3Dloader::scene::Node*> roots;
  tiny3Dloader::Importer importer;
  EXPECT_FALSE(importer.load("models/MissingBin.gltf", "models/", roots));

}

int main(int argc, char** argv)  {

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
