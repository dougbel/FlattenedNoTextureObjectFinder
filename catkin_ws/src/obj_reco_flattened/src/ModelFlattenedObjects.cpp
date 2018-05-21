/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2018  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "obj_reco_flattened/ModelFlattenedObjects.h"

 using namespace std;

ModelObjectsFlattened::ModelObjectsFlattened(){
	
}


bool ModelObjectsFlattened::setObject(string name, string nameMethodSegmentation,Scalar minScalarSegmentation,Scalar maxScalarSegmentation, float averageAreaPixels ){
	names.push_back(name);
	namesMethodsSegmentation.push_back(nameMethodSegmentation);
	vector <Scalar> scalars;
	scalars.push_back(minScalarSegmentation);
	scalars.push_back(maxScalarSegmentation);
	min_maxScalarsSegmentation.push_back(scalars);
	averagesAreasPixels.push_back(averageAreaPixels);
	
	
	if(nameMethodSegmentation=="HSV"){
		methodsSegmentation.push_back(Segmenter::colorSegmentHSV);
	}
	else if(nameMethodSegmentation=="HLS"){
		methodsSegmentation.push_back(Segmenter::colorSegmentHLS);
	}
	else if(nameMethodSegmentation=="BGR"){
		methodsSegmentation.push_back(Segmenter::colorSegmentBGR);
	}
	else{
		methodsSegmentation.push_back(Segmenter::colorSegmentHSV);
	}
			

}


bool ModelObjectsFlattened::loadKnowledgeBase(string path_file){
	loadKnowledgeBase();
}

bool ModelObjectsFlattened::loadKnowledgeBase(){
	Scalar minScalar;
	Scalar maxScalar;
	
	//AZULES --- HSV
	minScalar = Scalar(100, 162, 130);
	maxScalar = Scalar(107, 239, 219);
	setObject("dish", "HSV", minScalar,maxScalar,14000 );
	setObject("bowl", "HSV", minScalar, maxScalar,6000 );
	setObject("glass", "HSV", minScalar, maxScalar,3800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );
	//NARANJAS TRANSPARENTES --- HSV
	minScalar = Scalar(3, 119, 184);
	maxScalar = Scalar(16, 189, 255);
	setObject("dish", "HSV", minScalar,maxScalar, 14000 );
	setObject("bowl", "HSV", minScalar, maxScalar,6000 );
	setObject("glass", "HSV", minScalar, maxScalar,3800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );
	//NARAGNJAS --- BGR
	minScalar = Scalar(0, 0, 206);
	maxScalar = Scalar(8, 53, 243);
	setObject("dish", "BGR", minScalar,maxScalar, 14000 );
	setObject("bowl", "BGR", minScalar, maxScalar,6000 );
	setObject("glass", "BGR", minScalar, maxScalar,3800 );
	setObject("cutlery", "BGR", minScalar, maxScalar,800 );
	setObject("cutlery", "BGR", minScalar, maxScalar,1600 );
	
	//ROJO transparentes --- HSV
	minScalar = Scalar(165, 235, 162);
	maxScalar = Scalar(172, 255, 198);
	setObject("dish", "HSV", minScalar,maxScalar,14000 );
	setObject("bowl", "HSV", minScalar, maxScalar,6000);
	setObject("glass", "HSV", minScalar, maxScalar,3800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );
	//ROJOS BGR
	minScalar = Scalar(0, 0, 143);
	maxScalar = Scalar(112, 14, 204);
	setObject("dish", "BGR", minScalar,maxScalar,14000 );
	setObject("bowl", "BGR", minScalar, maxScalar,6000);
	setObject("glass", "BGR", minScalar, maxScalar,3800 );
	setObject("cutlery", "BGR", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );
	
	//MORADOS --- HSV
	minScalar = Scalar(150, 119, 84);
	maxScalar = Scalar(165, 255, 169);
	setObject("dish", "HSV", minScalar,maxScalar,14000 );
	setObject("bowl", "HSV", minScalar, maxScalar,6000);
	setObject("glass", "HSV", minScalar, maxScalar,3800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );
	//AMARILLOS --- HSV
	minScalar = Scalar(26, 244, 154);
	maxScalar = Scalar(28, 255, 255);
	setObject("dish", "HSV", minScalar,maxScalar,14000 );
	setObject("bowl", "HSV", minScalar, maxScalar,6000);
	setObject("glass", "HSV", minScalar, maxScalar,3800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,800 );
	setObject("cutlery", "HSV", minScalar, maxScalar,1600 );

}


int ModelObjectsFlattened::size(){
	return names.size();
}