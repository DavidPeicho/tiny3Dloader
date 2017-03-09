#include "main.hpp"

namespace tiny3Dloader {

  void
  tests::testReturnStatus(const char* file) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    EXPECT_TRUE(importer.load(file, "models/", scenes));

  }

  TEST(FilesValidity, MissingBinaryFile) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    EXPECT_FALSE(importer.load("models/MissingBin.gltf", "models/", scenes));

  }

  TEST(LanternFile, CheckStatus) {

    tests::testReturnStatus("models/Lantern.gltf");

  }

  TEST(LanternFile, TreeStructure) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    bool status = importer.load("models/Lantern.gltf", "models/", scenes);

    EXPECT_TRUE(status);

    // Checks the number of scenes
    EXPECT_EQ(1, scenes.size());
    // Checks the number of nodes in the scene
    const auto& scenePtr = scenes[0];
    EXPECT_EQ(1, scenePtr->nodes.size());
    // Checks the root node
    const auto& rootPtr = scenePtr->nodes[0];
    EXPECT_EQ(3, rootPtr->children.size());

    for (const auto& nodePtr : rootPtr->children) {
      EXPECT_EQ(0, nodePtr->children.size());
    }

  }

  TEST(LanternFile, Transforms) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    importer.load("models/Lantern.gltf", "models/", scenes);

    const auto& rootPtr = scenes[0]->nodes[0];

    EXPECT_EQ(0, rootPtr->children[0]->matrix.size());
    EXPECT_EQ(0, rootPtr->children[1]->matrix.size());
    EXPECT_EQ(0, rootPtr->children[2]->matrix.size());

    const auto& t0 = rootPtr->children[0]->translation;
    const auto& t1 = rootPtr->children[1]->translation;
    const auto& t2 = rootPtr->children[2]->translation;

    std::vector<float> expectedT0 = { -3.82315421f, 13.01603f, 0.0f };
    std::vector<float> expectedT1 = { -9.582001f, 21.0378723f, 0.0f };
    std::vector<float> expectedT2 = { -9.582007f, 18.0091515f, 0.0f };

    const auto& floatComparator = [](float a, float b) -> bool {

      if (fabs(a - b) <= 0.01f) return true;
      return false;

    };

    EXPECT_TRUE(std::equal(t0.begin(), t0.end(), expectedT0.begin(),
                           floatComparator));
    EXPECT_TRUE(std::equal(t1.begin(), t1.end(), expectedT1.begin(),
                           floatComparator));
    EXPECT_TRUE(std::equal(t2.begin(), t2.end(), expectedT2.begin(),
                           floatComparator));

  }

  TEST(BoomBoxFile, CheckStatus) {

    tests::testReturnStatus("models/BoomBox.gltf");

  }

  TEST(BoomBoxFile, TreeStructure) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    importer.load("models/BoomBox.gltf", "models/", scenes);

    EXPECT_EQ(1, scenes.size());

    const auto& rootPtr = scenes[0]->nodes[0];

    EXPECT_EQ(0, rootPtr->children.size());
    EXPECT_EQ("BoomBox", rootPtr->name);

  }

  TEST(BoomBoxFile, Transforms) {

    std::vector<tiny3Dloader::scene::Scene*> scenes;
    tiny3Dloader::Importer importer;
    importer.load("models/BoomBox.gltf", "models/", scenes);

    const auto& rootPtr = scenes[0]->nodes[0];

    const auto& floatComparator = [](float a, float b) -> bool {

      if (fabs(a - b) <= 0.01f) return true;
      return false;

    };

    const auto& t0 = rootPtr->translation;
    const auto& s0 = rootPtr->scale;
    std::vector<float> expectedT0 = { 0.0, 0.163, 0.053 };
    std::vector<float> expectedS0 = { 80.0, 80.0, 80.0 };


    EXPECT_TRUE(std::equal(t0.begin(), t0.end(), expectedT0.begin(),
                           floatComparator));
    EXPECT_TRUE(std::equal(s0.begin(), s0.end(), expectedS0.begin(),
                           floatComparator));

  }

}

int
main(int argc, char** argv)  {

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
