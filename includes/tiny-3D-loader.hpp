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

#define TYPE_BYTE (5020)
#define TYPE_UBYTE (5021)
#define TYPE_SHORT (5022)
#define TYPE_USHORT (5023)
#define TYPE_FLOAT (5026)

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
      load(std::string pathToFile, std::string assetsFolderPath = "") = 0;

      void
      freeScene();

    protected:
      std::string                             assetsFolderPath_;

      std::unordered_map<uint, scene::Node*>  nodesMap_;
  };

  class glTFLoader : Loader {

    public:
      scene::Node*
      load(std::string pathToFile, std::string assetsFolderPath = "") override;

    private:
      void
      processNode(uint nodeId);

      void
      processMesh(uint nodeId, uint meshId);

      void
      processBuffer(uint accessorId);

      void
      registerBuffer(uint bufferId);

    private:
      nlohmann::json                          json_;
      std::unordered_map<uint, char*>         binaryFiles_;

  };

  scene::Node*
  glTFLoader::load(std::string pathToFile, std::string assetsFolderPath) {

    std::ifstream stream(pathToFile);
    stream >> this->json_;

    this->assetsFolderPath_ = assetsFolderPath;

    // TODO: Check if every sections are created

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

    auto& jsonMesh = this->json_["meshes"][meshId];
    auto& jsonPrimitives = jsonMesh["primitives"];

    for (const auto& jsonPrimitive : jsonPrimitives) {
      const auto &attributes = jsonPrimitive["attributes"];

      uint normalId = attributes["NORMAL"].get<uint>();
      uint positionId = attributes["POSITION"].get<uint>();

      processBuffer(normalId);
      //const auto& normalAccessor = jsonAccessors[normalId];
      //const auto& positionAccessor = jsonAccessors[positionId];
      /*uint i = 0;
      while (attributes.find("TEXCOORD_" + i) != attributes.end()) {

      }*/
      //std::cout << normalAccessor << std::endl;

    }
  }

  void
  glTFLoader::processBuffer(uint accessorId) {

    const auto& jsonAccessors = this->json_["accessors"];
    const auto& jsonBufferViews = this->json_["bufferViews"];

    const auto& accessor = jsonAccessors[accessorId];

    uint bufferViewId = accessor["bufferView"];
    const auto& bufferView = jsonBufferViews[bufferViewId];

    uint bufferId = bufferView["buffer"].get<uint>();

    uint accessorBytesOffset = accessor["byteOffset"];
    uint bufferViewBytesOffset = bufferView["byteOffset"];

    if (this->binaryFiles_.find(bufferId) == this->binaryFiles_.end()) {
      std::cout << "Not save -> " << std::endl;
      registerBuffer(bufferId);
    }

  }

  void
  glTFLoader::registerBuffer(uint bufferId) {

    const auto& jsonBuffers = this->json_["buffers"];
    const auto& jsonBuffer = jsonBuffers[bufferId];

    std::string uri = this->assetsFolderPath_ + jsonBuffer["uri"].get<std::string>();
    std::ifstream ifs(uri, std::ios::in
                      | std::ios::binary
                      | std::ios::ate);

    // TODO: Handles binary file not read

    std::ifstream::pos_type fileSize = ifs.tellg();
    char* buffer = new char[fileSize];

    ifs.seekg(0, std::ios::beg);
    ifs.read(buffer, fileSize);

    this->binaryFiles_[bufferId] = buffer;
  }

} // namespace tiny3Dloader

