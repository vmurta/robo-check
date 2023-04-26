#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

int main() {
    string byuFileName = "robot.g";
    string objFileName = "robot.obj";

    // Open the BYU file for reading
    ifstream byuFile(byuFileName.c_str());
    if (!byuFile.is_open()) {
        cerr << "Error: Unable to open input file " << byuFileName << endl;
        return 1;
    }

    // Open the OBJ file for writing
    ofstream objFile(objFileName.c_str());
    if (!objFile.is_open()) {
        cerr << "Error: Unable to open output file " << objFileName << endl;
        return 1;
    }

    // Read the header
    int numParts, numVerts, numPolys, numEdges;
    byuFile >> numParts >> numVerts >> numPolys >> numEdges;

    cout << "numparts is " << numParts << "\n numVerts is " << numVerts << "\n numPolys is "
        << numPolys << "\n numEdges is " << numEdges;

    // Read the polygon indices for each part
    vector<pair<int, int>> partPolyIndices(numParts);
    for (int i = 0; i < numParts; ++i) {
        int startPoly, endPoly;
        byuFile >> startPoly >> endPoly;
        partPolyIndices[i] = make_pair(startPoly - 1, endPoly - 1);
    }

    // Read the vertex list
    vector<vector<float>> vertexList(numVerts, vector<float>(3));
    for (int i = 0; i < numVerts; ++i) {
        byuFile >> vertexList[i][0] >> vertexList[i][1] >> vertexList[i][2];
        objFile << "v " << vertexList[i][0] << " " << vertexList[i][1] << " " << vertexList[i][2] << endl;
    }


    // Read the polygon list
    cout << "got here before the loop" << endl;

    string line;
    getline(byuFile, line);
    cout << line << endl;  
    
    vector<int> current_polygon;
    for (int part = 0; part < numParts; ++part) {
        // for (int polyIndex = partPolyIndices[part].first; polyIndex <= 100; ++polyIndex) {
        for (int polyIndex = partPolyIndices[part].first; polyIndex <= partPolyIndices[part].second; ++polyIndex) {
            // cout << "got here to start of loop " << endl;
            string line;
            getline(byuFile, line);
            // if (line.length() == 0)
            //     continue;
            // cout << "got here" << endl;

            // cout << line << endl;  
            // continue;
            istringstream iss(line);

            // get vertices of current face
            // .g (BYU) files terminate their list of vertices by having the last
            // index be negative as a flag
            int vIndex;
            iss >> vIndex;
            while (vIndex >= 0) {
                current_polygon.push_back(vIndex -1);
                iss >> vIndex;
                // cout << "vIndex is" << vIndex << endl;
            }
            current_polygon.push_back(-vIndex -1);

            objFile << "f ";

            for ( int i = 0; i< current_polygon.size(); i++){
                objFile << current_polygon[i]<< " ";
            }

            objFile << endl;\
            current_polygon.clear();
        }
    }

    // Close the files
    byuFile.close();
    objFile.close();

    return 0;
}
