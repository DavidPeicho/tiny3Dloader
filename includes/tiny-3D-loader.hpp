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

#pragma once

#include <json.hpp>

#include <iostream>
#include <fstream>
#include <string>

#include <unordered_map>

namespace tiny3Dloader {

  namespace scene {

    struct Attributes {
      std::vector<float> vertices;
      std::vector<float> normals;
      std::vector<float> texcoords;
    };

    struct Mesh {
      std::string name;
      Attributes  attributes;
    };

    struct Node {
      std::string         name;
      std::vector<Mesh>   meshes;
      std::vector<Node*>  children;
    };

  } // namespace scene

  class Loader {

    public:
      virtual scene::Node*
      load(std::string pathToFile) = 0;

    protected:
      std::vector<scene::Node*> processedNodes;
  };

  class glTFLoader : Loader {

    public:
      scene::Node*
      load(std::string pathToFile) override;

    private:
      void
      processNode(uint nodeId);

      void
      processMesh(uint nodeId, uint meshId);

    private:
      nlohmann::json                          json_;
      std::unordered_map<uint, scene::Node*>  nodesMap_;

  };

  scene::Node*
  load(std::string pathToFile);

  scene::Node*
  load(std::string pathToFile) {

    glTFLoader loader;
    loader.load(pathToFile);

  }

  scene::Node*
  glTFLoader::load(std::string pathToFile) {

    std::ifstream stream(pathToFile);
    stream >> this->json_;

    uint nodeId = 0;
    auto& sceneNodes = this->json_["nodes"];

    // Allocates every nodes and add
    // them to the unordered map.
    for (const auto& jsonNode : sceneNodes) {
      scene::Node* node = new scene::Node;
      this->nodesMap_[nodeId] = node;

      ++nodeId;
    }

    nodeId = 0;
    for (const auto& jsonNode : sceneNodes) {
        processNode(nodeId);
        ++nodeId;
    }
  }

  void
  glTFLoader::processNode(uint nodeId) {

    auto& jsonNode = this->json_["nodes"][nodeId];
    auto& node = nodesMap_[nodeId];

    if (jsonNode.find("name") != jsonNode.end()) {
      node->name = jsonNode["name"];
    }

    if (jsonNode.find("mesh") != jsonNode.end()) {
      processMesh(nodeId, jsonNode["mesh"]);
    }

    // Handles parenting by linking
    // the children to the current nodes.
    if (jsonNode.find("children") != jsonNode.end()) {
      for (const auto childId : jsonNode["children"]) {
        processMesh(nodeId, childId);
      }
    }
  }

  void
  glTFLoader::processMesh(uint nodeId, uint meshId) {

  }

} // namespace tiny3Dloader

