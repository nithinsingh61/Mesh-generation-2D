
////////////////////////////////////////////////Global////////////////////////
#include "global_datatype.h"
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Point_2.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_map>
#include <sstream>
#include <time.h>
#include <cmath>
//End of Header Files

typedef CGAL::Point_2<Rep>									Point_1;
typedef Delaunay::Edge_iterator                     		Edge_iterator;
typedef Delaunay::Edge_circulator                     		Edge_circulator;
typedef my_K::Point_2		                     		    Point_2;
typedef my_K::Vector_2		                     		    Vector_2;
typedef my_K::Segment_2		                     		    Segment_2;
typedef my_K::Ray_2										    Ray_2;
typedef my_K::Direction_2									Direction_2;
typedef my_K::Iso_rectangle_2								Iso_rectangle_2;
//End of Typedef's

int points_inserted = 0;
int MAX_NUM=10;

//Class Node used for making quadtree
class Node
{
public:
	Node();
	
	Node(const Point_2& bottom,const Point_2& top);
	Node(const Iso_rectangle_2& box);
	virtual ~Node(){}

	std::vector<Point_2> insidePoints;
	Iso_rectangle_2 rectangle;
	Point_2 minCoor,maxCoor;

//private:
	std::shared_ptr<Node> parent;
	std::vector<std::shared_ptr<Node> > child;

};

Node::Node():minCoor(0,0),maxCoor(0,0),rectangle(minCoor,maxCoor){}
Node::Node(const Point_2& bottom,const Point_2& top):minCoor(bottom),maxCoor(top),rectangle(bottom,top){}
Node::Node(const Iso_rectangle_2& box):rectangle(box),minCoor(box.min()),maxCoor(box.max()){}

//Quad Tree class
class QuadTree
{
public:
	QuadTree();
	QuadTree(const Point_2& bottom,const Point_2& top);
	QuadTree(const Iso_rectangle_2& box);
	virtual ~QuadTree(){}

	//bool is_leafNode();
	bool is_leafNode(const std::shared_ptr<Node>& node);
	void spilt(std::shared_ptr<Node>& node);
	void insertPoints(std::shared_ptr<Node>& node);
	int checkNumPoints(const Node& node);

	//Point_2 leftDown,rightUp;
	std::shared_ptr<Node> rootNode;
	std::shared_ptr<Node> leafNode;
};

QuadTree::QuadTree(){}
QuadTree::QuadTree(const Point_2& bottom,const Point_2& top){rootNode->maxCoor=top;rootNode->minCoor= Point_2(bottom);}
QuadTree::QuadTree(const Iso_rectangle_2& box):rootNode(new Node(box)){}


bool QuadTree::is_leafNode(const std::shared_ptr<Node>& node)
{
	return node->child.size()==0;
}

void QuadTree::spilt(std::shared_ptr<Node>& node) {


	//CounterClockWise arrangement of Children startting from left bottom
	Point_2 midPoint((node->minCoor.x()+node->maxCoor.x())/2,(node->minCoor.y()+node->maxCoor.y())/2);

	Iso_rectangle_2 r1(node->minCoor.x(),node->minCoor.y(),midPoint.x(),midPoint.y());
	Iso_rectangle_2 r2(midPoint.x(),node->minCoor.y(),node->maxCoor.x(),midPoint.y());
	Iso_rectangle_2 r3(midPoint.x(),midPoint.y(),node->maxCoor.x(),node->maxCoor.y());
	Iso_rectangle_2 r4(node->minCoor.x(),midPoint.y(),midPoint.x(),node->maxCoor.y());

	node->child.push_back(std::shared_ptr<Node>(new Node(r1)));
	node->child.push_back(std::shared_ptr<Node>(new Node(r2)));
	node->child.push_back(std::shared_ptr<Node>(new Node(r3)));
	node->child.push_back(std::shared_ptr<Node>(new Node(r4)));

	for(int i =0;i<node->child.size();++i) {
		node->child.at(i)->parent=node;
	}
}

//Function to remove the duplicate points
void removeDup(std::shared_ptr<Node>& node)
{
	for(int i=0;i<node->insidePoints.size();++i)
		{
			for(int j=0;j<node->parent->insidePoints.size();++j)
			{
				if(node->insidePoints.at(i)==node->parent->insidePoints.at(j))
				{
					node->parent->insidePoints.erase(node->parent->insidePoints.begin(),node->parent->insidePoints.begin()+j);	
					break;
				}
			}
		//	std::cout<<node->parent->insidePoints.size()<<" Parent size"<<std::endl;
		}
}

//Function to insert points in the node of quadtree
void QuadTree::insertPoints(std::shared_ptr<Node>& node)
{
	if(node->parent!=NULL) 
	{
		for(int i=0;i<node->parent->insidePoints.size();++i) {
			if((node->rectangle.has_on_boundary(node->parent->insidePoints.at(i)))||(node->rectangle.has_on_bounded_side(node->parent->insidePoints.at(i)))) {
				node->insidePoints.push_back(node->parent->insidePoints.at(i));
			}
		}
	}

	else{std::cout<<"This is a root Node"<<std::endl;}
}


//Funtion to check number of points in the node
int QuadTree::checkNumPoints(const Node& node)
{
	return node.insidePoints.size();
}

//Function to generate quadtree
std::shared_ptr<Node> generate_QuadTree(QuadTree& tree,std::shared_ptr<Node>& node) {
	std::shared_ptr<Node> temp;
	if(node->insidePoints.size()>=MAX_NUM) {
		tree.spilt(node);
		
		for(int i=0;i<node->child.size();++i) {
			tree.insertPoints(node->child.at(i));
			
			temp=node->child.at(i);
			while(temp->insidePoints.size()>=MAX_NUM) {
				temp= generate_QuadTree(tree,temp);
			}
		}
	}
	return temp;
}

//Printing out the node of the quadtree
void printNode(std::shared_ptr<Node>& node) {
	if(node!=nullptr) {
		std::cout<<node->rectangle.vertex(0)<<" ";
		std::cout<< node->rectangle.vertex(1)<<std::endl;
		std::cout<< node->rectangle.vertex(1)<<" ";
		std::cout<< node->rectangle.vertex(2)<<std::endl;
		std::cout<< node->rectangle.vertex(2)<<" ";
		std::cout<< node->rectangle.vertex(3)<<std::endl;
		std::cout<< node->rectangle.vertex(3)<<" ";
		std::cout<< node->rectangle.vertex(0)<<std::endl;
	}
	else return;
}

//Function to print the quadtree in form of segments of boxes to display using opengl
void printTree(const QuadTree& tree) {
	std::queue<std::shared_ptr<Node>> nodeQueue;
	std::shared_ptr<Node> temp;
	if (tree.rootNode==nullptr){return;}
	else {
		nodeQueue.push(tree.rootNode);
		while(!nodeQueue.empty()){
			temp=nodeQueue.front();
			printNode(temp);
			nodeQueue.pop();
			for(int i=0;i<temp->child.size();++i){
				nodeQueue.push(temp->child.at(i));
			}
		}
	}
}




//Function to get the minimum and Maximum Coordinates of the Point Set
void MinMaxCoor(std::ifstream &input,Iso_rectangle_2& box){
	//double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	string str;
	while((std::getline(input,str))){
        Point_2 point;
		istringstream stream(str);
        while((stream>>point)){
			if(minX>point.x()){minX=point.x();}
			if(minY>point.y()){minY=point.y();}
			if(maxX<point.x()){maxX=point.x();}
			if(maxY<point.y()){maxY=point.y();}
        }
    }
	//std::cout<<maxX<<" "<<minX<<" "<<maxY<<" "<<minY<<std::endl;
	maxX=maxX+100;maxY=maxY+100;minX=minX-100;minY=minY-100;
	Iso_rectangle_2 bbox(minX,minY,maxX,maxY);
	box=bbox;
}

//Funtion to Create Delaunay Triangulation from a file of points
void create_Delaunay(Delaunay& dt,std::ifstream &input){
	std::istream_iterator <Point_2> begin(input);
   	std::istream_iterator <Point_2> end;
   	dt.insert(begin,end);
}

//To assign ids to the Delaunay Triangulation
void Assign_ids(Delaunay& dt){
	int id = 0;
	Finite_vertices_iterator_2d vit;
	for (vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
		vit->id = id;
		++id;
	}
}

//To output Delaunay Triangulation in OFF format to view in MeshView/Geomview
void OFF_format(Delaunay& dt,std::ofstream &output){
	//get the number of vertices and faces
	int vertex_count = dt.number_of_vertices();
	int face_count = dt.number_of_faces();
	int correct_edge_count =0;
	Finite_vertices_iterator_2d vit;

	if(output.is_open()){
		output<<"OFF"<<std::endl<<vertex_count<<" "<<face_count<<" "<<"0"<<std::endl;
		for (vit = dt.finite_vertices_begin(); vit!= dt.finite_vertices_end(); ++vit){
			output<<vit->point()<<" 0 "<< std::endl;
		}
	}

	for(fit f1 =dt.finite_faces_begin(); f1 != dt.finite_faces_end(); ++f1){
		vh first = (f1->vertex(0));
		unsigned int first_id = first->id;

		vh second = (f1->vertex(1));
		unsigned int second_id = second->id;

		vh third = (f1->vertex(2));
		unsigned int third_id = third->id;

		if(output.is_open()){
			output<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
			output<<f1->vertex(0)->point()<<" " <<f1->vertex(1)->point()<<" " <<f1->vertex(2)->point()<<std::endl;
		}
	}
  	output.close();
}
void OFF_formatInside(Delaunay& dt,std::ofstream &output){
	//get the number of vertices and faces

	int vertex_count = dt.number_of_vertices();
	int face_count = dt.number_of_faces();
	int correct_edge_count =0;
	Finite_vertices_iterator_2d vit;

	if(output.is_open()){
		output<<"OFF"<<std::endl<<vertex_count<<" "<<face_count<<" "<<"0"<<std::endl;
		for (vit = dt.finite_vertices_begin(); vit!= dt.finite_vertices_end(); ++vit){
			output<<vit->point()<<" 0 "<< std::endl;
		}
	}

	fit f1 =dt.finite_faces_begin();
	for(; f1 != dt.finite_faces_end(); ++f1){
		if(f1->face_is_inside){
			vh first = (f1->vertex(0));
			unsigned int first_id = first->id;

			vh second = (f1->vertex(1));
			unsigned int second_id = second->id;

			vh third = (f1->vertex(2));
			unsigned int third_id = third->id;

			if(output.is_open()){
				output<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
			}
		}
	}
	output.close();
}

//Function to Output Delaunay Edges so as to view in Opengl
void outputDelaunayEdges(Delaunay dt,std::ofstream &output){
	for(fit f1 =dt.finite_faces_begin(); f1 != dt.finite_faces_end(); ++f1){
		vh first = (f1->vertex(0));
		vh second = (f1->vertex(1));
		vh third = (f1->vertex(2));

		if(output.is_open()){
			output<<first->point()<<" ";output<<second->point()<<std::endl;
			output<<second->point()<<" ";output<<third->point()<<std::endl;
			output<<third->point()<<" ";output<<first->point()<<std::endl;
		}
	}
  	output.close();
}

//Function to Output Delaunay Edges so as to view in Opengl
void outputInsideDelaunayEdges(Delaunay dt,std::ofstream &output){
	fit f1 =dt.finite_faces_begin();

	for(; f1 != dt.finite_faces_end(); ++f1){
		if(f1->face_is_inside==true){
			vh first = (f1->vertex(0));
			vh second = (f1->vertex(1));
			vh third = (f1->vertex(2));

			if(output.is_open()){
				output<<first->point()<<" ";output<<second->point()<<std::endl;
				output<<second->point()<<" ";output<<third->point()<<std::endl;
				output<<third->point()<<" ";output<<first->point()<<std::endl;
			}
		}
	}
  	output.close();
}




//Funtion to get the points lying inside the slab of spcified thickness/threshold and store it in an array
/*template <typename Type1,typename Type2>
void thresh_Points(const Type1& Object1,const Type2& Object2,float threshold,std::vector<Type2>& Object3){
	if(CGAL::squared_distance(Object1,Object2)<=threshold){
		Object3.push_back(Object2);
	}
}*/

//funtion to Extend/Limit the rays to the Bounding Box(The first box in the QuadTree)
//template <typename Type1>
Segment_2 convToSeg(const Iso_rectangle_2& box,const Ray_2& ray){

	CGAL::Object obj = CGAL::intersection(ray,box);
	//std::cout<<ray<<" Ray in convtoseg"<<std::endl;

	const Point_2* tempPoint=CGAL::object_cast<Point_2>(&obj);
	const Segment_2* tempSeg=CGAL::object_cast<Segment_2>(&obj);
	Segment_2 seg;

	if(tempPoint!=nullptr){
		//std::cout<<" In point convtoseg"<<std::endl;
		Segment_2 temp(ray.source(),*tempPoint);
		seg=temp;
		return seg;
	}
	if(tempSeg!=nullptr){
		//std::cout<<" In segment convtoseg"<<std::endl;
		seg=*tempSeg;
		return seg;
	}
	
	//std::cout<<seg<<" Ray in convtoseg"<<std::endl;
}

//funtion to get a bounding box given a ray(converted to a segment) or a segment for a given threshold value
std::vector<Segment_2> bBox(const Segment_2& edge,const int& thresh)
{
	double dist=thresh;
	dist=std::sqrt(dist);

	Segment_2 tempSeg(Point_2(0,0),Point_2(0,0));
	Direction_2 dir=edge.direction();

	if(dir.dx()==0)
	{
		
		Point_2 bottomL(edge.source().x(),edge.source().y()-dist);
		Point_2 bottomR(edge.source().x(),edge.source().y()+dist);
		Point_2 topR(edge.target().x(),edge.target().y()+dist);
		Point_2 topL(edge.target().x(),edge.target().y()-dist);

		Segment_2 temp(bottomL,topL);
		Segment_2 temp1(bottomR,topR);
		std::vector<Segment_2> tempVec;
		tempVec.reserve(3);
		tempVec.push_back(temp);
		tempVec.push_back(temp1);
		tempVec.push_back(edge);
		
		return tempVec;
	}
	
	else
	{
		double slope=dir.dy()/dir.dx();
		
		Point_2 bottomL(edge.source().x()-dist*std::cos(std::atan(-1/slope)),edge.source().y()-dist*std::sin(std::atan(-1/slope)));
		Point_2 topR(edge.target().x()+dist*std::cos(std::atan(-1/slope)),edge.target().y()+dist*std::sin(std::atan(-1/slope)));
		Point_2 bottomR(edge.source().x()+dist*std::cos(std::atan(-1/slope)),edge.source().y()+dist*std::sin(std::atan(-1/slope)));
		Point_2 topL(edge.target().x()-dist*std::cos(std::atan(-1/slope)),edge.target().y()-dist*std::sin(std::atan(-1/slope)));

		Segment_2 temp(bottomL,topL);
		Segment_2 temp1(bottomR,topR);
		std::vector<Segment_2> tempVec;
		tempVec.reserve(3);
		tempVec.push_back(temp);
		tempVec.push_back(temp1);
		tempVec.push_back(edge);
		

		return tempVec;
	}

}

void thresh_Points(const Segment_2&  seg,const Point_2 point,const int& thresh,std::vector<Point_2>& threshPoints)
{
	
	std::vector<Segment_2> threshSeg=bBox(seg,thresh);
	//extendSegments(box,threshSeg);
	threshSeg.erase(threshSeg.begin()+2);
	
	//std::cout<<"thresh_Points "<<seg<<" "<<point<<" "<<thresh<<" "<<threshSeg.size()<<std::endl;

	/*for(int i=0;i<2;++i)
	{
		std::cout<<threshSeg.at(i)<<std::endl;
	}*/

	/*threshSeg.push_back(Segment_2(threshSeg.at(0).target(),threshSeg.at(1).target()));
	threshSeg.push_back(Segment_2(threshSeg.at(1).source(),threshSeg.at(0).source()));

	Ray_2 ray=Ray_2(point,box.vertex(0));

	int count=check_Intersection(ray,threshSeg);
	
	if(count%2!=0)
	{
		threshPoints.push_back(point);
	}*/
	

	bool insideThresh=false;
	Vector_2  AP=Vector_2(threshSeg.at(0).source(),point);
	Vector_2  AB=Vector_2(threshSeg.at(0));
	Vector_2  AD=Vector_2(threshSeg.at(0).source(),threshSeg.at(1).source());

	if((AP*AB>=0)&&(AP*AB<AB*AB))
	{
		if((AP*AD>=0)&&(AP*AD<AD*AD))
		{
			insideThresh=true;
		}
	}
	if(insideThresh==true)
	{
		threshPoints.push_back(point);
	}

	//threshSeg.at(1)=Segment_2(threshSeg.at(1).target(),threshSeg.at(1).source());
	
	/*std::cout<<"thresh_Points "<<threshSeg.size()<<std::endl;
	for(int i=0;i<4;++i)
	{
		std::cout<<threshSeg.at(i)<<std::endl;
	}*/


	/*bool current=false,prev=false,insideThresh=false;
	for(int i=0;i<4;++i)
	{
		if(CGAL::angle(threshSeg.at(i).source(),threshSeg.at(i).target(),point)==CGAL::ACUTE)
		{
			current=false;
		}
		else
		{
			current=true;
		}
		//std::cout<<current<<" Current"<<std::endl;
		//std::cout<<current<<std::endl;
		if(i!=0)
		{
			if(current!=prev)
			{
				insideThresh=false;
				break;
			}
			if(current==prev)
			{
				if(current==false)
				{
					insideThresh=true;
				}

				else
				{
					insideThresh=false;
				}
			
			}
		}
		prev=current;
	}*/

	
/*	for(int i=0;i<threshPoints.size();++i)
	{
		std::cout<<threshPoints.at(i)<<" thresh Points "<<thresh<<std::endl;
	}*/
	//std::cout<<"thresh_Points1 "<<threshPoints.size()<<std::endl;
}

//Function to get the insidePoints given tree, voronoi edge and threshold 
std::vector<Point_2> insidePoints(const QuadTree& tree,const Segment_2& edge,const int& thresh){
	std::vector<std::vector<Point_2> > tempPoints;
	tempPoints.reserve(5);
	std::queue<std::shared_ptr<Node>> nodeQueue;
	std::unordered_map<Point_2,int> mapping;
	//int count=0;
	
	std::vector<Point_2> th_Points;
	th_Points.reserve(10);
	std::shared_ptr<Node> temp;
	//temp.reserve(4);
	
	
	std::vector<Segment_2> tempSeg=bBox(edge,thresh);
	
	
	if (tree.rootNode==nullptr){return th_Points;}
	else{
	
		nodeQueue.push(tree.rootNode);
		while(!nodeQueue.empty()){
			temp=nodeQueue.front();
			
			if(temp->child.size()==0)
			{
				tempPoints.push_back(temp->insidePoints);
				/*for(int i=0;i<temp->insidePoints.size();++i)
				{
					std::unordered_map<Point_2,int>::const_iterator got = mapping.find (temp->insidePoints.at(i));
					

					if (got == mapping.end())
					{
						mapping.insert(std::make_pair<Point_2,int>(temp->insidePoints.at(i),mapping.size()));
					}
					
  
				}*/
			}

			for(int i=0;i<temp->child.size();++i){
			
				bool boxInside=true;
				if((CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(0))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(1))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(2))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(3))<=thresh))
				{
					boxInside=true;
				}
				else
				{
					boxInside=false;
				}

				if((CGAL::do_intersect(tempSeg.at(0),temp->child.at(i)->rectangle))||(CGAL::do_intersect(tempSeg.at(1),temp->child.at(i)->rectangle))||(CGAL::do_intersect(tempSeg.at(2),temp->child.at(i)->rectangle))||(boxInside==true)){
					nodeQueue.push(temp->child.at(i));
				}
				
			}
			nodeQueue.pop();
		}
	}
	for(int i=0;i<tempPoints.size();++i){
		for(int j=0;j<tempPoints[i].size();++j){
			thresh_Points(edge,tempPoints[i].at(j),thresh,th_Points);
		}
	}
	std::vector<Point_2> th1_Points;
	th1_Points.reserve(10);
	std::cout<<th_Points.size()<<" Original"<<std::endl;
	/*for( std::unordered_map<Point_2,int>::iterator it=mapping.begin();it!=mapping.end();++it)
	{
		//std::cout<<(*it)->first<<std::endl;
		thresh_Points(edge,it->first,thresh,th1_Points);
	}
	std::cout<<th1_Points.size()<<" Dup"<<std::endl;*/
	return th_Points;
}

//Function to get nearest and next_nearest neighbors using NNCrust and store it in a vector in the form {point,nearest,next_nearest}
template <typename Type>
void NNCrust(Delaunay& dt,std::vector<Point_2>& sample,std::vector<vector <Point_2> >& Neighbors,std::vector<Segment_2>& seg,Type ray,int threshold ){
		Finite_vertices_iterator_2d vit;
	std::vector< vector <Point_2> > TestNeigh;

	//std::cout<<" NNCRUST"<<std::endl;
	
	for(vit = dt.finite_vertices_begin();vit!=dt.finite_vertices_end();++vit)
	{
		bool notThresh_point=true;
		//std::cout<<"NNCRUST"<<std::endl;
		int distThreshold=CGAL::squared_distance(ray,vit->point());
		if(distThreshold<threshold){
			notThresh_point=false;
		}
		else{
			notThresh_point=true;
		}

		if(notThresh_point==false)
		{
			int count = 0;

			//for each vertex find the Delaunay neighbors.
			std::vector<vh> neighbors;
			std::vector<Point_2> TestP;
			TestP.reserve(10);
			TestP.push_back(vit->point());

			//get a face handle incident on vit
			
			//std::cout<<" NNCRUST 1 "<<vit->point()<<std::endl;
			int index=0,index1=0;double d_min = DBL_MAX,d_min1=DBL_MAX;
			Vertex_circulator vcirculator=dt.incident_vertices(vit),done(vcirculator);
			//std::cout<<" NNCRUST 1.1"<<std::endl;
			do{
				if(!dt.is_infinite(vcirculator)){
			//std::cout<<" NNCRUST 1.1.0"<<std::endl;
					//neighbors.push_back(vcirculator);
					TestP.push_back(vcirculator->point());
					double d= squared_distance(TestP.at(0),vcirculator->point());
		//	std::cout<<" NNCRUST 1.1.1"<<std::endl;
					if(d<d_min){
						d_min=d;
						//std::cout<<" NNCRUST 1.1.2"<<std::endl;
						index=TestP.size()-1;
						//std::cout<<" NNCRUST 1.1.3"<<std::endl;
					}
				}
			} while(++vcirculator != done);
			//std::cout<<" NNCRUST 1.2"<<std::endl;
			//std::cout<<index<<" "<<TestP.size()<<std::endl;
			for(int i=1;i<TestP.size();++i)
			{
				if (((TestP.at(index) != TestP.at(i))&&(TestP.at(0)!=TestP.at(i))) && (CGAL::angle(TestP.at(index),TestP.at(0),TestP.at(i)) != CGAL::ACUTE))
				{
					double dist = squared_distance(TestP.at(0),TestP.at(i));
					if (dist < d_min1)
					{
						d_min1 = dist;
						index1=i;
					}
				}
			}
			//std::cout<<index1<<" "<<TestP.size()<<std::endl;
			//std::cout<<" NNCRUST 1.2"<<std::endl;
			//std::cout<<" NNCRUST5"<<std::endl;
			std::vector<Point_2> TempNeighbors;
			TempNeighbors.reserve(3);
			TempNeighbors.push_back(TestP.at(0));
			//std::cout<<" NNCRUST6"<<std::endl;
			//std::cout<<" NNCRUST 1.3"<<std::endl;
			TempNeighbors.push_back(TestP.at(index));
			//std::cout<<" NNCRUST7"<<std::endl;
			//std::cout<<" NNCRUST 1.4"<<std::endl;
			TempNeighbors.push_back(TestP.at(index1));
			//std::cout<<" NNCRUST 1.5"<<std::endl;
			Neighbors.push_back(TempNeighbors);
			/*for(int i=0;i<neighbors.size();++i)
			{
				TestP.push_back(neighbors[i]->point());
			}*/
			//std::cout<<" NNCRUST 2"<<std::endl;
			//TestNeigh.push_back(TestP);
		}
	}
	/*for(int l=0;l<TestNeigh.size();++l){
		int nearest=0;int next_nearest=0;
		
		/*for(int j=1;j<TestNeigh[l].size();++j){
			double d= squared_distance(TestNeigh[l].at(0),TestNeigh[l].at(j));
			if(d<d_min){
				d_min=d;
				nearest=j;
			}
		}
		std::cout<<" NNCRUST 3"<<std::endl;
		std::cout<<TestNeigh[l].size()<<" "<<index<<std::endl;
		for(int k=1;k<TestNeigh[l].size();++k){
			std::cout<<" NNCRUST 3.1"<<std::endl;
			Point p1 = TestNeigh[l].at(0);
			std::cout<<" NNCRUST 3.2"<<std::endl;
			Point p2 = TestNeigh[l].at(index);
			std::cout<<" NNCRUST3.3"<<std::endl;
			Point p3 = TestNeigh[l].at(k);
			double dist = squared_distance(p1,p3);
			std::cout<<" NNCRUST 4"<<std::endl;
			if (((p2 != p3)&&(p1!=p3)) && (CGAL::angle(p2,p1,p3) != CGAL::ACUTE)){

				if (dist < d_min1){
					d_min1 = dist;
					next_nearest=k;
				}
			}
		}
		std::cout<<" NNCRUST5"<<std::endl;
		std::vector<Point_2> TempNeighbors;
		TempNeighbors.push_back(TestNeigh[l].at(0));
		std::cout<<" NNCRUST6"<<std::endl;
		TempNeighbors.push_back(TestNeigh[l].at(index));
		std::cout<<" NNCRUST7"<<std::endl;
		TempNeighbors.push_back(TestNeigh[l].at(next_nearest));
		Neighbors.push_back(TempNeighbors);
	}*/
	//std::cout<<Neighbors.size()<<std::endl;
	//std::cout<<" NNCRUST8"<<std::endl;
	//std::cout<<Neighbors.size()<<" Neighbors NNCRUST"<<std::endl;
	for(int p=0;p<Neighbors.size();++p)
	{
		Segment_2 segment1(Neighbors[p].at(0),Neighbors[p].at(1));
		Segment_2 segment2(Neighbors[p].at(0),Neighbors[p].at(2));
		
		seg.push_back(segment1);
		seg.push_back(segment2);
	}
	//std::cout<<" NNCRUST9"<<std::endl;
	/*for(int m=0;m<seg.size();++m)
	{
		std::cout<<seg.at(m)<<std::endl;
	}*/
	//std::cout<<seg.size()<<" Before"<<std::endl;

	for(int q=0;q<seg.size();++q)
	{
		for(int r=q;r<seg.size();++r)
		{
			if(r==q){continue;}
			else
			{
				if(((seg.at(q).source()==seg.at(r).target())&&(seg.at(q).target()==seg.at(r).source())))
				{								
					//std::cout<<seg.at(r)<<std::endl;
					seg.erase(seg.begin()+r);
					--r;
				}
			}
		}
	}
	//std::cout<<" NNCRUST10"<<std::endl;
	/*if(seg.size()>0){
	for(int q=0;q<seg.size()-1;++q){
			for(int r=q;r<seg.size();++r){
				if(((seg.at(q).source()==seg.at(r).target())&&(seg.at(q).target()==seg.at(r).source()))||((seg.at(r).target().x()-seg.at(r).source().x()<0.05)&&(seg.at(r).target().y()-seg.at(r).source().y()<0.05))){
					
					seg.erase(seg.begin()+r);
				}
			}
	}
	}*/
	//std::cout<<seg.size()<<" After"<<std::endl;

	//std::cout<<seg.size()<<" Seg NNCRUST"<<std::endl;
	
	
}

//Function to check number of intersections
template <typename Type1,typename Type2>
int check_Intersection(Type1 rays,std::vector<Type2> seg){
	int count=0;
	bool on_theRay=false;

	for(int j=0;j<seg.size();++j){
		//std::cout<<seg.at(j)<<std::endl;
		if((seg.at(j).source().x()<DBL_MAX)||(seg.at(j).source().y()<DBL_MAX)||(seg.at(j).target().x()<DBL_MAX)||(seg.at(j).target().y()<DBL_MAX)){
			if(seg.at(j).source()!=seg.at(j).target()){
				if(CGAL::do_intersect(rays,seg.at(j))){
					++count;
				}
				//if((rays.has_on(seg.at(j).source()))||(rays.has_on(seg.at(j).target()))){--count;std::cout<<"It is here in check intersection"<<std::endl;}
			}
		}
	}
	//if(on_theRay){--count;}
	return count;
}



//Function to get the farthest point
template <typename Type1,typename Type2,typename Type3>
Point_2 get_Farthest_Point(Type1& ray,std::vector<Type2>& seg,Type3& point){
	Point_2 Point;
	double d_max=0;

	for(int i=0;i<seg.size();++i){
		Point_2 far_point;
		if((CGAL::do_intersect(ray,seg.at(i)))&&(seg.at(i).source()!=seg.at(i).target())){
			const Point_2* tempPoint;

			if(seg.at(i).is_vertical()){
				double t =(seg.at(i).source().x()-ray.source().x())/ray.direction().dx();
				Point_2 pq(seg.at(i).source().x(),ray.source().y()+t*(ray.direction().dy()));
				far_point=pq;
			}

			else{

				CGAL::Object obj = CGAL::intersection(ray,seg.at(i));
				tempPoint=CGAL::object_cast<Point_2>(&obj);
			
				if(tempPoint==NULL){std::cout<<"tempPoint is NULL "<<std::endl;std::cout<<seg.at(i)<<std::endl;}
				const Segment_2* tempSeg=CGAL::object_cast<Segment_2>(&obj);
				
				if(tempPoint){far_point=*tempPoint;}
				if(tempSeg){std::cout<<"It is a segment"<<std::endl;}
			}
				double dist=CGAL::squared_distance(point,far_point);
				if(dist>d_max){
					Point = far_point;
					d_max=dist;
				}
		}
		else{}
	}
	return Point;
}

bool isSkinnyTriangle(fh faceItr,int &skinny_bound_zero,int &skinny_bound_one,int &skinny_bound_two,int &skinny_bound_three)
{
	double circumradius = 0;
	double s_distance = 0;
	bool answer = false;

	vh first = (faceItr->vertex(0));
	unsigned int first_id = first->id;

	vh second = (faceItr->vertex(1));
	unsigned int second_id = second->id;

	vh third = (faceItr->vertex(2));
	unsigned int third_id = third->id;

	Point onePt = first->point();
	Point twoPt = second->point();
	Point threePt = third->point();

	//Find Circumradius
	circumradius = CGAL::squared_radius(onePt, twoPt, threePt);

	double dist1 = CGAL::squared_distance(onePt, twoPt);
	double dist2 = CGAL::squared_distance(twoPt, threePt);
	double dist3 = CGAL::squared_distance(threePt, onePt);

	//c is the circumcenter of the concerned triangle
	Point c = CGAL::circumcenter(onePt, twoPt, threePt);

	//s_distance is the smallest edge length
	if(dist1 < dist2 && dist1 < dist3)
		s_distance = dist1;
	else if (dist2 < dist1 && dist2 < dist3)
		s_distance = dist2;
	else
		s_distance = dist3;

	//if r/l ratio is greater than sqrt(2.0)
	/*if(circumradius/s_distance >= 0.2 && circumradius/s_distance < 0.5){++skinny_bound_zero;}

	if(circumradius/s_distance >= 0.5 && circumradius/s_distance < 1.0){
			++skinny_bound_one;
		}
	if(circumradius/s_distance >= 1.5 && circumradius/s_distance <= 2){
			skinny_bound_two++;
		}*/
	if(circumradius/s_distance > 2)				// SKINNY TRIANGLES
		{
			++skinny_bound_three;
			answer = true;
		}

	return answer;
}

bool is_circumcenter_inside(Point_1 p1, Point_1 p2, Point_1 p3, Point_1 q){
	if(CGAL::collinear(p1, p2, q) || CGAL::collinear(p2, p3, q) || CGAL::collinear(p3, p1, q)){
		return 1;}
	else if((CGAL::orientation(p1, p2, q) == CGAL::orientation(p2, p3, q)) && (CGAL::orientation(p2, p3, q) == CGAL::orientation(p3, p1, q))){
		return 1;}
	else{
		return 0;
	}
}

fh circumcenter_position(fh faceItrs, bool &circumcenter_is_outside,Point& circum_center)
{
		int not_to_check = -1;
		circumcenter_is_outside = true;
		fh next_face;
		bool move_ahead = true;

		vh first = (faceItrs->vertex(0));
		vh second = (faceItrs->vertex(1));
		vh third = (faceItrs->vertex(2));

		//Find centroid
		Point centroid = CGAL::centroid(first->point(), second->point(), third->point());
		Point c = CGAL::circumcenter(first->point(), second->point(), third->point());

		Point_1 p1(c.x(), c.y());
		Point_1 p2(centroid.x(), centroid.y());
		line_2d l(p1, p2);			// Line between centroid and circumcenter.

		while(move_ahead){
			Point_1 p_first(first->point().x(), first->point().y());
			Point_1 p_second(second->point().x(), second->point().y());
			Point_1 p_third(third->point().x(), third->point().y());

			line_2d line_1(p_first, p_second);  // Line between first and second point of triangle
			line_2d line_2(p_second, p_third);	// Line between second and third point of triangle
			line_2d line_3(p_third, p_first);	// Line between third and first point of triangle

			// FIRST CHECK IF CIRCUMCENTER IS INSIDE THIS TRIANGLE
			if(is_circumcenter_inside(p_first, p_second, p_third, p1)){
				if(!faceItrs->face_is_inside){
					circumcenter_is_outside = true;
				}
				else{
					circumcenter_is_outside = false;
				}
				move_ahead = false;
				break;
			}
			else{
				if(not_to_check==-1 || not_to_check != 2){
					if(CGAL::do_intersect(l, line_1) == true){
						next_face = faceItrs->neighbor(2);
					}
				}
				if (not_to_check==-1 || not_to_check != 0){
					if(CGAL::do_intersect(l, line_2) == true ){
						next_face = faceItrs->neighbor(0);
					}
				}
				if (not_to_check==-1 || not_to_check != 1){
					if(CGAL::do_intersect(l, line_3) == true ){
						next_face = faceItrs->neighbor(1);
					}
				}
				not_to_check = next_face->index(faceItrs);
				assert(next_face->neighbor(not_to_check) == faceItrs);
				faceItrs = next_face;
			}
			first = (faceItrs->vertex(0));
			second = (faceItrs->vertex(1));
			third = (faceItrs->vertex(2));
		}
		circum_center=c;
		return faceItrs;
}

void deFace(Delaunay& dt, const vh& point)
{
	/*all_fit a_fit;
	vh temp;
	temp->point()=point;
	std::cout<<temp->point()<<" deFace"<<std::endl;
	temp->id=0;
	a_fit=dt.all_faces_begin();

	Face_circulator fc=dt.incident_faces(temp),done(fc);

	//Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);
	do
	{
		for(int i = 0; i < 3; ++i)
		{
			fc->correct_segments[i] = false;
			fh fc1=fc->neighbor(temp);
		}
	}while(++fc!=done);
							
	//int triangle_i = 1;
	for ( ; a_fit !=dt.all_faces_end();++a_fit)
	{
		for(int i = 0; i < 3; ++i)
		{
			a_fit->correct_segments[i] = false;
		}
		//++triangle_i;
	}*/

	
	Face_circulator fc=dt.incident_faces(point),done(fc);

	//Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);
	do
	{
		//std::cout<<point->id<<" "<<point->point();
		for(int i = 0; i < 3; ++i)
		{
			//point->
			fc->correct_segments[i] = false;
			fh opp_face=fc->neighbor(i);

			int opp_index = opp_face->index(fc);
			opp_face->correct_segments[opp_index] = false;
		}
	}while(++fc!=done);

}

int main(){

	std::clock_t t1,t2;
	t1=clock();
	int stop;
	std::vector<Point_2> OriginalSample, RandomSample, threshPoints;
	std::srand(time(NULL));
	
	std::ifstream input("Points_xy_50000.txt");
	std::ofstream outputRandomSample("RandomSample.txt");
	
	int num_of_points=0;
	std::string data;
	
	
	while(std::getline(input,data)){
		Point_2 original;
		std::istringstream stream(data);
		while(stream >> original)		{
			OriginalSample.push_back(original);
			++num_of_points;
	 	}
	}      
	input.close();
	
		
	/*for(int i=0;i<OriginalSample.size();++i){
		double x=OriginalSample.at(i).x()*50;
		double y=OriginalSample.at(i).y()*50;
		Point_2 temp(x,y);
		OriginalSample.at(i)=temp;   
	}*/
	
	std::ifstream input2("Points_xy_50000.txt");
	//input.open("Flower_401");
	Delaunay dt;
	create_Delaunay(dt,input2);
	
	input2.close();
	Assign_ids(dt);
	
	std::ifstream input1("Points_xy_50000.txt");
	Iso_rectangle_2	boundingBox;
	MinMaxCoor(input1,boundingBox);

	QuadTree tree(boundingBox);
	tree.rootNode->insidePoints=OriginalSample;
	generate_QuadTree(tree,tree.rootNode);
	 // printTree(tree);
	
	
	//std::cout<<OriginalSample.size()<<" OriginalSample Size"<<std::endl;
	//std::cout<<"Input 1 to random and 0 not random"<<std::endl;
	int con_input=1;
	//std::cin>>con_input;
	
	if(con_input==1)
	{
		for(int i=0;i<4;++i)
		{			
			int n=std::rand()%(num_of_points-1);
			RandomSample.push_back(OriginalSample.at(n));
			if(outputRandomSample.is_open())
				outputRandomSample<<OriginalSample.at(n)<<std::endl;
		}
	}
	//std::cout<<"Random Point selection done"<<std::endl;
	std::ifstream inRandomSample;
	if(con_input==1){
		inRandomSample.open("RandomSample.txt");
	}
	
	if(con_input==0){
		inRandomSample.open("SpecialCase.txt");
	}
	
	Delaunay dt_sample;
	create_Delaunay(dt_sample,inRandomSample);
	
	Finite_vertices_iterator_2d vit;
		
	bool iterate=true;
	
	while(iterate){
		//std::cin>>stop;
		
		int number_of_skinny_triangle = 0;
		int skinny_bound_zero = 0;
		int skinny_bound_one = 0;
		int skinny_bound_two = 0;
		int skinny_bound_three = 0;
		int skinny_bound_is_two = 0;

		std::ofstream outputPoints;
		std::ofstream outputNN;
		outputNN.open("OutputNN.txt",std::ofstream::out | std::ofstream::trunc);
		
		std::ofstream outputOFF;
		outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
		
		std::ofstream outputDelaunay;
		outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
		outputDelaunayEdges(dt_sample,outputDelaunay);
		
		std::ofstream outputRays;
		outputRays.open("OutputRays.txt",std::ofstream::out | std::ofstream::trunc);
		
		std::ofstream HomoEdges;
		HomoEdges.open("HomomorphicalEdges.txt",std::ofstream::out | std::ofstream::trunc);

		bool rays_ok=true;
		bool segs_ok=true;

				         
		Edge_iterator eit =dt_sample.edges_begin();

		for ( ; eit !=dt_sample.edges_end();++eit){
			CGAL::Object o = dt_sample.dual(eit);

			//std::cout<<"Here "<<std::endl;
			const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
        	const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
			if (r){
				//outputRays<<*r<<std::endl;
				std::vector<Point_2> ThreshPoints;
				
				if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
					Segment_2 temp=convToSeg(tree.rootNode->rectangle,*r);
					ThreshPoints=insidePoints(tree,temp,1);
				}
				//std::cout<<"thresh is "<<ThreshPoints.size()<<std::endl;
				std::vector<vector <Point_2> > Neighbors;
				/*outputPoints.open("OutputThreshPoints.txt",std::ofstream::out | std::ofstream::trunc);
				
				for(int k=0;k<ThreshPoints.size();k++){
					outputPoints<<ThreshPoints.at(k)<<std::endl;
					
				}
				//std::cout<<ThreshPoints.size()<<" ThreshPoints Size before"<<std::endl;
				/*std::ifstream inThreshPoints("OutputThreshPoints.txt");
				Delaunay dt_thresh;
				create_Delaunay(dt_thresh,inThreshPoints);
				Assign_ids(dt_thresh);*/
				
				//outputPoints.close();
			//	std::ofstream outputThreshDelaunay;
				//outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			//	outputDelaunayEdges(dt,outputThreshDelaunay);
				//outputThreshDelaunay.close();
				
				std::vector<Segment_2> Neighbor_Segments;
				/*std::vector<Point_2> NewThreshPoints;
				for(int j=0;j<ThreshPoints.size();j++){
					thresh_Points(*r,ThreshPoints.at(j),300,NewThreshPoints);
				}*/
				
				
				
				for(int j=0;j<ThreshPoints.size();++j)
				{
					for(int k=j;k<ThreshPoints.size();++k)
					{
						if(j==k){continue;}
						else
						{
							if(ThreshPoints.at(j)==ThreshPoints.at(k))
							{
								//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
								ThreshPoints.erase(ThreshPoints.begin()+k);	--k;
							}
							
						}
					}
				}

				//std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;
				int num_of_intersections=0;
				
				if(ThreshPoints.size()>0){

					NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*r,1);
					//std::cout<<Neighbor_Segments.size()<<"Neoghbour Segments"<<std::endl;
					num_of_intersections=check_Intersection(*r,Neighbor_Segments);
				}
				
				//std::cout<<"no of intersection "<<num_of_intersections<<std::endl;
				
				
				if(num_of_intersections>1){
					iterate=true;
					rays_ok=false;
					
					Point_2 Far_Point=get_Farthest_Point(*r,Neighbor_Segments,eit->first->vertex((eit->second+1)%3)->point());
					//std::cout<<"Far_Point "<<Far_Point<<std::endl;
					if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;} 
					/*if(outputNN.is_open()) {
						for(int i=0;i<Neighbor_Segments.size();i++) {
							outputNN<<Neighbor_Segments.at(i)<<std::endl;
						}
						outputNN.close();
					}*/
					
					vh tempVhandle=dt_sample.insert(Far_Point);
					++points_inserted;
					deFace(dt_sample,tempVhandle);
					
					/*std::ofstream outputOFF;
					outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
					
					OFF_format(dt_sample,outputOFF);
					*/
					break;
				}
				
				if(num_of_intersections==1) {
					eit->first->correct_segments[eit->second]=true;
					//std::cout<<eit->first->vertex((eit->second)%3)->point()<<"is   "<<eit->first->vertex((eit->second+1)%3)->point()<<" \t "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
					//std::cout<<eit->first->vertex((eit->second+1)%3)->point()<<" \t "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
 					//HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
					fh opp_face = eit->first->neighbor(eit->second);
					
					int opp_index = opp_face->index(eit->first);
					opp_face->correct_segments[opp_index] = true;
					//std::cout<<opp_face->vertex((opp_index + 1)%3)->point()<<" "<<opp_face->vertex((opp_index + 2)%3)->point()<<std::endl;
				}
				//std::cin>>stop;
			}
        	
			if (s) {
      			
				std::vector<Point_2> ThreshPoints;
				ThreshPoints=insidePoints(tree,*s,1);
				std::vector<vector <Point_2> > Neighbors;
				//outputRays<<*s<<std::endl;
        		
				/*outputPoints.open("OutputThreshPoints.txt",std::ofstream::out | std::ofstream::trunc);
				for(int k=0;k<ThreshPoints.size();k++){
					outputPoints<<ThreshPoints.at(k)<<std::endl;
				}*/
				
				//std::ifstream inThreshPoints("OutputThreshPoints.txt");
				/*Delaunay dt_thresh;
				create_Delaunay(dt_thresh,inThreshPoints);
				Assign_ids(dt_thresh);*/
				//outputPoints.close();
				
				//std::ofstream outputThreshDelaunay;
				//outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
				//outputDelaunayEdges(dt_thresh,outputThreshDelaunay);
				//outputThreshDelaunay.close();
				
				std::vector<Segment_2> Neighbor_Segments;
				/*std::vector<Point_2> NewThreshPoints;
				for(int j=0;j<ThreshPoints.size();j++){
					thresh_Points(*s,ThreshPoints.at(j),300,NewThreshPoints);
				}*/

				for(int j=0;j<ThreshPoints.size();++j)
				{
					for(int k=j;k<ThreshPoints.size();++k)
					{
						if(j==k){continue;}
						else
						{
							if(ThreshPoints.at(j)==ThreshPoints.at(k))
							{
								//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
								ThreshPoints.erase(ThreshPoints.begin()+k);	--k;
							}
							
						}
					}
				}

				//std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;

				int num_of_intersections=0;
				if(ThreshPoints.size()>0){
					NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*s,1);
					num_of_intersections=check_Intersection(*s,Neighbor_Segments);
				}
				//std::cout<<"no of intersection "<<num_of_intersections<<std::endl;
				if(num_of_intersections>1) {
					iterate=true;
					segs_ok=false;
					
					Point_2 Far_Point=get_Farthest_Point(*s,Neighbor_Segments,eit->first->vertex((eit->second+1)%3)->point());
					//std::cout<<"Far_Point "<<Far_Point<<std::endl;
					if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;} 
						/*if(outputNN.is_open()){
							for(int i=0;i<Neighbor_Segments.size();i++){
								outputNN<<Neighbor_Segments.at(i)<<std::endl;
							}
							outputNN.close();
						}*/
						vh tempVhandle=dt_sample.insert(Far_Point);
						++points_inserted;
						deFace(dt_sample,tempVhandle);
						

						break;
					}
				
				if(num_of_intersections==1){
						eit->first->correct_segments[eit->second]=true;
						//std::cout<<eit->first->vertex((eit->second)%3)->point()<<"is   "<<eit->first->vertex((eit->second+1)%3)->point()<<" \t "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
						//std::cout<<eit->first->vertex((eit->second+1)%3)->point()<<" \t "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;					
						//HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
						fh opp_face = eit->first->neighbor(eit->second);
						int opp_index = opp_face->index(eit->first);
						opp_face->correct_segments[opp_index] = true;
						//std::cout<<opp_face->vertex((opp_index + 1)%3)->point()<<" "<<opp_face->vertex((opp_index + 2)%3)->point()<<std::endl;
					}
				}
				//Assign_ids(dt_sample);
			
			}
			//std::cout<<"Topology Part 1 done"<<std::endl;
			/*if((rays_ok!=true)||(segs_ok!=true)){
				all_fit a_fit;
				a_fit=dt_sample.all_faces_begin();
				int triangle_i = 1;
				for ( ; a_fit !=dt_sample.all_faces_end();++a_fit){
					for(int i = 0; i < 3; ++i){
						a_fit->correct_segments[i] = false;
					}
					++triangle_i;
				}
			}*/
			
			//HomoEdges.close();
			//outputRays.close();
			//std::cin>>stop;
				
			if((rays_ok==true)&&(segs_ok==true))
			{
				//std::cout<<"Check for Degree in topology"<<std::endl;
				iterate=false;
				std::ofstream outputDelaunay;
				outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
				outputDelaunayEdges(dt_sample,outputDelaunay);
				outputDelaunay.close();
/*
				std::ofstream outputOFF;
				outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
	
				OFF_format(dt_sample,outputOFF);*/
				
				eit=dt_sample.edges_begin();
				for ( ; eit !=dt_sample.edges_end();++eit) {

					if(eit->first->correct_segments[eit->second]==true)
					{
						for(int i = 1; i < 3; ++i){
							
							int count=0;
							vh vh_center = eit->first->vertex((eit->second+i)%3);
							
							Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);
							do{
								if(circulator->first->correct_segments[circulator->second]==true){
									++count;
								}
							}while(++circulator!=done);
							
							if((count>2)){
								iterate=true;
								Point_2 Farthest_Point =vh_center->point();
								
								Edge_circulator circulator1=dt_sample.incident_edges(vh_center),done(circulator1);
								do{
									if(circulator1->first->correct_segments[circulator1->second]==true)
									{
										CGAL::Object o = dt_sample.dual(circulator1);
										const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
										const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
    		  							Point_2 Far_Point;
       									
										if(r){
											//outputRays<<*r<<std::endl;
        									std::vector<Point_2> ThreshPoints,NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;
        									
											if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
												Segment_2 temp=convToSeg(tree.rootNode->rectangle,*r);
												ThreshPoints=insidePoints(tree,temp,1);
											}

											/*outputPoints.open("OutputThreshPoints.txt",std::ofstream::out | std::ofstream::trunc);
											for(int k=0;k<ThreshPoints.size();k++){
												outputPoints<<ThreshPoints.at(k)<<std::endl;
											}*/
											
											/*std::ifstream inThreshPoints("OutputThreshPoints.txt");
											Delaunay dt_thresh;
											create_Delaunay(dt_thresh,inThreshPoints);
											Assign_ids(dt_thresh);
											std::ofstream outputThreshDelaunay;
											outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);

											//outputDelaunayEdges(dt_thresh,outputThreshDelaunay);
											outputThreshDelaunay.close();*/
											//outputPoints.close();
											
											std::vector<Segment_2> Neighbor_Segments;

											/*for(int j=0;j<ThreshPoints.size();j++){
												thresh_Points(*r,ThreshPoints.at(j),300,NewThreshPoints);
											}*/
			
											for(int j=0;j<ThreshPoints.size();++j)
											{
												for(int k=j;k<ThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(ThreshPoints.at(j)==ThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															ThreshPoints.erase(ThreshPoints.begin()+k);	
															--k;
														}
							
													}
												}
											}

											//std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;
											
											NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*r,1);
											//std::cout<<"Neighbor Segment size is "<<Neighbor_Segments.size()<<std::endl;
											Far_Point=get_Farthest_Point(*r,Neighbor_Segments,vh_center->point());

										/*	if((CGAL::squared_distance(Far_Point, v1) < 5) || (CGAL::squared_distance(Far_Point, v2) < 5) || (CGAL::squared_distance(Far_Point, v3) < 5) || (CGAL::squared_distance(Far_Point, v4) < 5))
										{
											std::cout<<"Close Inserted Point _Ray"<<std::endl;
										}*/
										if((CGAL::squared_distance(vh_center->point(),Far_Point)) > (CGAL::squared_distance(vh_center->point(),Farthest_Point)))
										{
        									Farthest_Point=Far_Point;
        								}

											//std::cout<<"Far_Point "<<Far_Point<<std::endl;
											/*outputNN.open("OutputNN.txt",std::ofstream::out | std::ofstream::trunc);
											
											if(outputNN.is_open()){
												for(int i=0;i<Neighbor_Segments.size();i++){
													outputNN<<Neighbor_Segments.at(i)<<std::endl;
												}
												outputNN.close();
											}*/

        								}
        								
										if(s){
        									std::vector<Point_2> ThreshPoints=insidePoints(tree,*s,1),NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;
											//outputRays<<*s<<std::endl;
										/*	outputPoints.open("OutputThreshPoints.txt",std::ofstream::out | std::ofstream::trunc);
											for(int k=0;k<ThreshPoints.size();++k){
												outputPoints<<ThreshPoints.at(k)<<std::endl;
											}*/
											
											/*std::ifstream inThreshPoints("OutputThreshPoints.txt");
											Delaunay dt_thresh;
											create_Delaunay(dt_thresh,inThreshPoints);
											Assign_ids(dt_thresh);*/
											outputPoints.close();
											
											//std::ofstream outputThreshDelaunay;
											//outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
											//outputDelaunayEdges(dt_thresh,outputThreshDelaunay);
											//outputThreshDelaunay.close();
											
											std::vector<Segment_2> Neighbor_Segments;
										/*	for(int j=0;j<ThreshPoints.size();++j){
												thresh_Points(*s,ThreshPoints.at(j),300,NewThreshPoints);
											}*/
											
											for(int j=0;j<ThreshPoints.size();++j)
											{
												for(int k=j;k<ThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(ThreshPoints.at(j)==ThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															ThreshPoints.erase(ThreshPoints.begin()+k);	
															--k;
														}
							
													}
												}
											}

										//	std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;
				
											NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*s,1);
										//	std::cout<<"Neighbor Segment size is "<<Neighbor_Segments.size()<<std::endl;
										
											Far_Point=get_Farthest_Point(*s,Neighbor_Segments,vh_center->point());
											//std::cout<<"Far_Point "<<Far_Point<<std::endl;
											/*outputNN.open("OutputNN.txt",std::ofstream::out | std::ofstream::trunc);
											if(outputNN.is_open())
											{
												for(int i=0;i<Neighbor_Segments.size();i++)
												{
													outputNN<<Neighbor_Segments.at(i)<<std::endl;
												}
												outputNN.close();
											}*/
        								}
										
										if((CGAL::squared_distance(vh_center->point(),Far_Point)) > (CGAL::squared_distance(vh_center->point(),Farthest_Point)))
										{
        									Farthest_Point=Far_Point;
        								}
        							}
								}while(++circulator1!=done);
								
								
								vh tempVhandle=dt_sample.insert(Farthest_Point);
								++points_inserted;
								deFace(dt_sample,tempVhandle);
								
								if(outputRandomSample.is_open()){outputRandomSample<<Farthest_Point<<std::endl;}
								break;
							}
						}
					}
					if(iterate==true){break;}
				}
				
				if(iterate==false) {
					
					std::cout<<"Geometric"<<std::endl;
					//std::cin>>stop;

					std::ofstream outputDelaunay;
					outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
					outputDelaunayEdges(dt_sample,outputDelaunay);
					outputDelaunay.close();
					//Assign_ids(dt_sample);
					/*std::ofstream outputOFF;
					outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
	
					OFF_format(dt_sample,outputOFF);*/
					
					//std::ofstream outputThreshDelaunay;
					//outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
					
					//eit=dt_sample.edges_begin();
					//for(; eit !=dt_sample.edges_end();++eit) 
					//{
					//	if(eit->first->correct_segments[eit->second]==true) 
					//	{
					//		outputThreshDelaunay<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point() <<std::endl;
					//	}
						
					//}
					//outputThreshDelaunay.close();
										
					eit=dt_sample.edges_begin();
					//std::cout<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
					
					for ( ; eit !=dt_sample.edges_end();++eit) 
					{
						if(eit->first->correct_segments[eit->second]==true) 
						{
							
							for(int i = 1; i < 3; ++i)
							{
								int count=0;int p=0;
								vh vh_center = eit->first->vertex((eit->second+i)%3);
							//	std::cout<<vh_center->point()<<" VhCEnter"<<std::endl;
								
								vh vh_adj,vh_adj1;
								std::vector<Edge_circulator> edge_cir;
								Edge_circulator circulator_edge;
								Edge_circulator circulator_edge1;
								Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);
							
								do{
									if(circulator->first->correct_segments[circulator->second]==true)
										++p;
								}while(++circulator !=done);
								
								do{
								//std::cout<<"1"<<std::endl;
									if(circulator->first->correct_segments[circulator->second]==true){
										if(count<1){circulator_edge=circulator;}
										if(count>=1){circulator_edge1=circulator;}
										if(vh_center==circulator->first->vertex((circulator->second+1)%3)){
											//std::cout<<"vh_center"<<vh_center->point()<<std::endl;
											if(count<1){vh_adj=circulator->first->vertex((circulator->second+2)%3);
											//std::cout<<"vh_adj"<<vh_adj->point()<<std::endl;
											}
											if(count>=1){vh_adj1=circulator->first->vertex((circulator->second+2)%3);
											//std::cout<<"vh_adj1"<<vh_adj1->point()<<std::endl;
											}
										}
								
										if(vh_center==circulator->first->vertex((circulator->second+2)%3)){
											//std::cout<<"vh_center"<<vh_center->point()<<std::endl;
											if(count<1){vh_adj=circulator->first->vertex((circulator->second+1)%3);
											//std::cout<<"vh_adj"<<vh_adj->point()<<std::endl;
											}
											if(count>=1){vh_adj1=circulator->first->vertex((circulator->second+1)%3);
											//std::cout<<"vh_adj1"<<vh_adj1->point()<<std::endl;
											}
										}
										
										edge_cir.push_back(circulator);
										++count;
									}
								}while(++circulator!=done);
								//std:cin>>stop;

								//std::cout<<"Squared distance "<<CGAL::squared_distance(vh_center->point(), vh_adj->point())<<"Squared distance 2"<<CGAL::squared_distance(vh_center->point(), vh_adj1->point())<<std::endl;
								
								if((CGAL::squared_distance(vh_center->point(), vh_adj->point()) < 3) && (CGAL::squared_distance(vh_center->point(), vh_adj1->point()) < 3))
								{	edge_cir.clear();
								//std::cout<<"close in geometry"<<std::endl;
									break;
								}
								//std::cout<<"2"<<std::endl;
								
         						double vec1=(vh_center->point().x()-vh_adj->point().x())*(vh_center->point().x()-vh_adj1->point().x());
								double vec2=(vh_center->point().y()-vh_adj->point().y())*(vh_center->point().y()-vh_adj1->point().y());
								double denom=(std::sqrt(((vh_center->point().x()-vh_adj->point().x())*(vh_center->point().x()-vh_adj->point().x()))+(vh_center->point().y()-vh_adj->point().y())*(vh_center->point().y()-vh_adj->point().y())))*(std::sqrt(((vh_center->point().x()-vh_adj1->point().x())*(vh_center->point().x()-vh_adj1->point().x()))+(vh_center->point().y()-vh_adj1->point().y())*(vh_center->point().y()-vh_adj1->point().y())));
								double ang;
								
								if(denom!=0){ang=std::acos((vec1+vec2)/denom);}
								
								if(ang<(160*3.14)/180)
								{									
  									double dist=CGAL::squared_distance(edge_cir.at(0)->first->vertex((edge_cir.at(0)->second+1)%3)->point(),edge_cir.at(0)->first->vertex((edge_cir.at(0)->second+2)%3)->point());
									double dist1=CGAL::squared_distance(edge_cir.at(1)->first->vertex((edge_cir.at(1)->second+1)%3)->point(),edge_cir.at(1)->first->vertex((edge_cir.at(1)->second+2)%3)->point());
								
									Edge_circulator circulator1;
									if(dist<=dist1){circulator1=edge_cir.at(1);}
									if(dist>dist1){circulator1=edge_cir.at(0);}
									if(circulator1==NULL){std::cout<<"It is NULL "<<std::endl;}
									
									CGAL::Object o = dt_sample.dual(circulator1);
									const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
									const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
    		  						Point_2 Far_Point;
       								if(r)
									{
										std::vector<Point_2> ThreshPoints,NewThreshPoints;
        								std::vector<vector <Point_2> > Neighbors;
        									
											if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
												Segment_2 temp=convToSeg(tree.rootNode->rectangle,*r);
												ThreshPoints=insidePoints(tree,temp,1);
											}

											
										//	outputPoints.close();
											
											std::vector<Segment_2> Neighbor_Segments;

											
			
											for(int j=0;j<ThreshPoints.size();++j)
											{
												for(int k=j;k<ThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(ThreshPoints.at(j)==ThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															ThreshPoints.erase(ThreshPoints.begin()+k);	
															--k;
														}
							
													}
												}
											}

											//std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;
											
											NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*r,1);
											//std::cout<<"Neighbor_Segments size is  "<<Neighbor_Segments.size()<<std::endl;
										Far_Point=get_Farthest_Point(*r,Neighbor_Segments,vh_center->point());
										
									}
        							
									if(s)
									{
										//std::cout<<"Geo Seg"<<std::endl;
        								std::vector<Point_2> ThreshPoints=insidePoints(tree,*s,1),NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;

										
											//outputPoints.close();
											
											//std::ofstream outputThreshDelaunay;
											//outputThreshDelaunay.open("NewDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
											//outputDelaunayEdges(dt_thresh,outputThreshDelaunay);
											//outputThreshDelaunay.close();
											
											std::vector<Segment_2> Neighbor_Segments;
									
											
											for(int j=0;j<ThreshPoints.size();++j)
											{
												for(int k=j;k<ThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(ThreshPoints.at(j)==ThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															ThreshPoints.erase(ThreshPoints.begin()+k);	
															--k;
														}
							
													}
												}
											}

											//std::cout<<ThreshPoints.size()<<" ThreshPoints Size after"<<std::endl;
				
											NNCrust(dt,ThreshPoints,Neighbors,Neighbor_Segments,*s,1);
										//std::cout<<"Neighbor_Segments size is  "<<Neighbor_Segments.size()<<std::endl;
										Far_Point=get_Farthest_Point(*s,Neighbor_Segments,vh_center->point());
										
									}
									
									vh tempVhandle=dt_sample.insert(Far_Point);
									++points_inserted;
									deFace(dt_sample,tempVhandle);


									if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;} 
									++points_inserted;
									
								iterate=true;
								break;
							}
							edge_cir.clear();
						
						}
					}
					if(iterate==true){break;}
				}
			}
		}
		if(iterate==false){
			std::cout<<"Mesh Refinement"<<std::endl;
			std::ofstream outputDelaunay;
			outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);

			outputDelaunayEdges(dt_sample,outputDelaunay);
			outputDelaunay.close();

		//	Assign_ids(dt_sample);
			/*std::ofstream outputOFF;
			outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
	
			OFF_format(dt_sample,outputOFF);*/
			
			int inside_faces_count = 0;
			int vertex_count=0;
			fh f_handle = dt_sample.incident_faces(dt_sample.infinite_vertex());
			
			std::stack<fh> fh_stack;
			fh_stack.push(f_handle);
			
			double shortest_edge = DBL_MAX;
			double shortest_length = 0;
			int no_of_triangle = 0;
			
			while(fh_stack.empty())
			{				
				break;
			}

			do
			{
				f_handle = fh_stack.top();
				//std::f_handle->vertex()
				fh_stack.pop();
			
				if(f_handle->face_is_inside == true)
				{
					f_handle->face_is_inside = false;
					
					for(int i = 0; i < 3; ++i)
					{
						if(f_handle->correct_segments[i]==false)
						{
							fh_stack.push(f_handle->neighbor(i));
						}
						else{}
					}
				}
			}while(!fh_stack.empty());
			
			Finite_faces_iterator_2d face_iterator;
			for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); ++face_iterator) 
			{
				if(face_iterator->face_is_inside == true)
				{
					++inside_faces_count;
				}
			}
			int debug_counter= 0;
			std::ofstream outputDelaunayInside;
			outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
			//std::cout<<"dt_sample faces "<<dt_sample.number_of_faces()<<std::endl;
			for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); ++face_iterator)
			{
				
				Point circum_center;
				bool skinny_triangle = false;
				bool Circumcenter_is_outside = false;
			
				if(face_iterator->face_is_inside == true)
				{//std::cout<<"face is inside"<<std::endl;
					skinny_triangle = isSkinnyTriangle(face_iterator,skinny_bound_zero,skinny_bound_one,skinny_bound_two,skinny_bound_three);
					if(skinny_triangle)
					{
						++number_of_skinny_triangle;
						int skinny;
						
						fh containing_face = circumcenter_position(face_iterator, Circumcenter_is_outside,circum_center);
						if (!Circumcenter_is_outside)
						{
							dt_sample.insert(circum_center);
							if(outputRandomSample.is_open()){outputRandomSample<<circum_center<<std::endl;}
							++points_inserted;
							all_fit a_fit1;
							a_fit1=dt_sample.all_faces_begin();
							
							int triangle_i1 = 1;
							for ( ; a_fit1 !=dt_sample.all_faces_end();++a_fit1)
							{
								for(int i2 = 0; i2 < 3; ++i2)
								{
									a_fit1->correct_segments[i2] = false;
								}
								a_fit1->face_is_inside=true;
								++triangle_i1;
							}
							iterate=true;
							break;
						}
					}
					++no_of_triangle;
				}
			}
			std::cout<<"Total number of triangles = "<<no_of_triangle<<std::endl;
		}
		if(iterate==false){
			std::cout<<"number of skinny triangles "<<number_of_skinny_triangle<<std::endl;
		}
	}

	std::ofstream outputOFF;
	outputOFF.open("off.off",std::ofstream::out | std::ofstream::trunc);
	
	OFF_formatInside(dt_sample,outputOFF);
	std::cout<<"Number of inserted points are = "<< points_inserted<<std::endl;
	t2=clock();
	float timeSec=((float)(t2)-(float)(t1))/CLOCKS_PER_SEC;
	std::cout<<"Time Taken for Execution: "<<timeSec<<std::endl;
	int k;
	std::cin>>k;
	
	return 0;
}
