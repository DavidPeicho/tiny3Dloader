/*
 * Copyright (c) 2017 David Peicho

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "obj.hpp"

namespace tiny3Dloader {

namespace tests {

TEST(Quadt, TreeStructure) {

  std::vector<std::shared_ptr<tiny3Dloader::scene::Scene>> scenes;
  tiny3Dloader::Importer importer;
  bool status = importer.load("models/obj/Quad-t.obj", scenes);

  EXPECT_TRUE(status);

  // Checks the number of scenes
  EXPECT_EQ(1, scenes.size());
  // Checks the number of nodes in the scene
  const auto& scenePtr = scenes[0];
  EXPECT_EQ(1, scenePtr->nodes.size());

  const auto& rootPtr = scenePtr->nodes[0];
  EXPECT_EQ(0, rootPtr->children.size());

  // Checks number of meshes
  EXPECT_EQ(1, rootPtr->meshes.size());

}

TEST(Quadt, Content) {

  std::vector<std::shared_ptr<tiny3Dloader::scene::Scene>> scenes;
  tiny3Dloader::Importer importer;
  bool status = importer.load("models/obj/Quad-t.obj", scenes);

  const auto& rootPtr = scenes[0]->nodes[0];
  const auto& primitives = rootPtr->meshes[0]->primitives;

  EXPECT_EQ("Plane", rootPtr->name);

  EXPECT_EQ(1, primitives.size());
  // Checks number of vertices
  EXPECT_EQ(12, primitives[0]->vertices.size());
  // Checks number of vertices
  EXPECT_EQ(12, primitives[0]->normals.size());
  // Checks number of vertices
  EXPECT_EQ(6, primitives[0]->indices.size());

  std::vector<float> expectedVertices = {
     1.0f, 0.0f, 1.0f,
    -1.0f, 0.0f, -1.0f,
    -1.0f, 0.0f, 1.0f,
     1.0f, 0.0f, -1.0f
  };
  std::vector<float> expectedNormals = {
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f
  };
  std::vector<unsigned int> expectedIndices = {
    0, 1, 2, 0, 3, 1
  };

  EXPECT_TRUE(utils::vectorEquality(primitives[0]->vertices, expectedVertices));
  EXPECT_TRUE(utils::vectorEquality(primitives[0]->normals, expectedNormals));
  EXPECT_TRUE(utils::vectorEquality(primitives[0]->indices, expectedIndices));

}

TEST(Quadq, TreeStructure) {

  std::vector<std::shared_ptr<tiny3Dloader::scene::Scene>> scenes;
  tiny3Dloader::Importer importer;
  bool status = importer.load("models/obj/Quad-q.obj", scenes);

  EXPECT_TRUE(status);

  // Checks the number of scenes
  EXPECT_EQ(1, scenes.size());
  // Checks the number of nodes in the scene
  const auto& scenePtr = scenes[0];
  EXPECT_EQ(1, scenePtr->nodes.size());

  const auto& rootPtr = scenePtr->nodes[0];
  EXPECT_EQ(0, rootPtr->children.size());

  // Checks number of meshes
  EXPECT_EQ(1, rootPtr->meshes.size());

}

TEST(Quadq, Content) {

  std::vector<std::shared_ptr<tiny3Dloader::scene::Scene>> scenes;
  tiny3Dloader::Importer importer;
  bool status = importer.load("models/obj/Quad-q.obj", scenes);

  EXPECT_TRUE(status);

  const auto& rootPtr = scenes[0]->nodes[0];
  const auto& primitives = rootPtr->meshes[0]->primitives;

  EXPECT_EQ("Plane", rootPtr->name);

  EXPECT_EQ(1, primitives.size());
  // Checks number of vertices
  EXPECT_EQ(12, primitives[0]->vertices.size());
  // Checks number of vertices
  EXPECT_EQ(12, primitives[0]->normals.size());
  // Checks number of vertices
  EXPECT_EQ(6, primitives[0]->indices.size());

  std::vector<float> expectedVertices = {
    1.0f, 0.0f, 1.0f,
    -1.0f, 0.0f, -1.0f,
    -1.0f, 0.0f, 1.0f,
    1.0f, 0.0f, -1.0f
  };
  std::vector<float> expectedNormals = {
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f
  };
  std::vector<unsigned int> expectedIndices = {
    0, 1, 2, 0, 3, 1
  };

  EXPECT_TRUE(utils::vectorEquality(primitives[0]->vertices, expectedVertices));
  EXPECT_TRUE(utils::vectorEquality(primitives[0]->normals, expectedNormals));
  EXPECT_TRUE(utils::vectorEquality(primitives[0]->indices, expectedIndices));

}

TEST(Cubet, Content) {

  std::vector<std::shared_ptr<tiny3Dloader::scene::Scene>> scenes;
  tiny3Dloader::Importer importer;
  bool status = importer.load("models/obj/Cube-t.obj", scenes);

  EXPECT_TRUE(status);

  const auto& rootPtr = scenes[0]->nodes[0];
  const auto& primitives = rootPtr->meshes[0]->primitives;

  EXPECT_EQ("", rootPtr->name);
  EXPECT_EQ("cube", rootPtr->meshes[0]->name);

  EXPECT_EQ(1, primitives.size());
  // Checks number of vertices
  EXPECT_EQ(72, primitives[0]->vertices.size());
  // Checks number of vertices
  EXPECT_EQ(72, primitives[0]->normals.size());
  // Checks number of vertices
  EXPECT_EQ(36, primitives[0]->indices.size());

}

} // namespace tests

} // namespace tiny3Dloader
