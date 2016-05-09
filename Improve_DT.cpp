#define _CRT_SECURE_NO_WARNINGS

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

///Start of Various functions



int points_inserted = 0;
int MAX_NUM=20;

class Node
{
public:
	Node();
	//Node(float xmin,float ymin,float xmax,float ymax);
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
//QuadTree::~QuadTree(){}
QuadTree::QuadTree(const Point_2& bottom,const Point_2& top){rootNode->maxCoor=top;rootNode->minCoor= Point_2(bottom);}
QuadTree::QuadTree(const Iso_rectangle_2& box):rootNode(new Node(box)){}


bool QuadTree::is_leafNode(const std::shared_ptr<Node>& node)
{
	return node->child.size()==0;
}

void QuadTree::spilt(std::shared_ptr<Node>& node)
{
	
	//if(QuadTree::is_leafNode(node))
	{
		//std::cout<<node->maxCoor<<"Spilt Node"<<std::endl;
		//CounterClockWise arrangement of Children startting from left bottom
		Point_2 midPoint((node->minCoor.x()+node->maxCoor.x())/2,(node->minCoor.y()+node->maxCoor.y())/2);
		//std::cout<<"Mid Point "<<midPoint<<std::endl;
		
		Iso_rectangle_2 r1(node->minCoor.x(),node->minCoor.y(),midPoint.x(),midPoint.y());
		Iso_rectangle_2 r2(midPoint.x(),node->minCoor.y(),node->maxCoor.x(),midPoint.y());
		Iso_rectangle_2 r3(midPoint.x(),midPoint.y(),node->maxCoor.x(),node->maxCoor.y());
		Iso_rectangle_2 r4(node->minCoor.x(),midPoint.y(),midPoint.x(),node->maxCoor.y());
		
		//std::cout<<"Spilt 1"<<std::endl;
		

		//Node n1(r1),n2(r2),n3(r3),n4(r4);
		//n1.rectangle=r1;n2.rectangle=r2;n3.rectangle=r3;n4.rectangle=r4;
		node->child.push_back(std::shared_ptr<Node>(new Node(r1)));
		node->child.push_back(std::shared_ptr<Node>(new Node(r2)));
		node->child.push_back(std::shared_ptr<Node>(new Node(r3)));
		node->child.push_back(std::shared_ptr<Node>(new Node(r4)));
		//std::cout<<"Spilt 2"<<std::endl;

		//std::cout<<n1.rectangle<<std::endl;
		//std::cout<<node->child.at(0)->rectangle<<" Checking "<<std::endl;

		for(int i =0;i<node->child.size();++i){node->child.at(i)->parent=node;}
		//std::cout<<"Spilt 3"<<std::endl;
		//for(int i=0;i<node->child.size();++i){delete node->child.at(i);node.child.at(i)=NULL;}
		
	}
	//else{std::cout<<"Cannot Split Unless The Node is a Leaf Node"<<std::endl;}

}

void QuadTree::insertPoints(std::shared_ptr<Node>& node)
{
	if(node->parent!=NULL)
	{
		for(int i=0;i<node->parent->insidePoints.size();++i)
		{
			if((node->rectangle.has_on_boundary(node->parent->insidePoints.at(i)))||(node->rectangle.has_on_bounded_side(node->parent->insidePoints.at(i))))
			{
				node->insidePoints.push_back(node->parent->insidePoints.at(i));
				//////////ensure you delete the points in the nodes after split
			}
		}
	}
	else{std::cout<<"This is a root Node"<<std::endl;}
}

int QuadTree::checkNumPoints(const Node& node)
{
	return node.insidePoints.size();
}

std::shared_ptr<Node> generate_QuadTree(QuadTree& tree,std::shared_ptr<Node>& node)
{
	std::shared_ptr<Node> temp;
	//std::cout<<node->insidePoints.size()<<std::endl;
	if(node->insidePoints.size()>=MAX_NUM)
	{
		tree.spilt(node);
		for(int i=0;i<node->child.size();++i)
		{
			tree.insertPoints(node->child.at(i));
			temp=node->child.at(i);
			//////////ensure you delete the points in the nodes after split

			while(temp->insidePoints.size()>=MAX_NUM)
			{
				temp= generate_QuadTree(tree,temp);
			}
		}
	}
	return temp;
}

void printNode(std::shared_ptr<Node>& node)
{
	if(node!=nullptr)
	{
		std::cout<<node->rectangle.vertex(0)<<" ";
		std::cout<< node->rectangle.vertex(1)<<std::endl;
		std::cout<< node->rectangle.vertex(1)<<" ";
		std::cout<< node->rectangle.vertex(2)<<std::endl;
		std::cout<< node->rectangle.vertex(2)<<" ";
		std::cout<< node->rectangle.vertex(3)<<std::endl;
		std::cout<< node->rectangle.vertex(3)<<" ";
		std::cout<< node->rectangle.vertex(0)<<std::endl;

		std::cout<<node->child.size()<<"Size////////////////////"<<std::endl;
		std::cout<<node->insidePoints.size()<<"//////////////////////Inside Points"<<std::endl;
	}
	else return;
}

void printTree(const QuadTree& tree)
{
	std::queue<std::shared_ptr<Node>> nodeQueue;
	std::shared_ptr<Node> temp;
	if (tree.rootNode==nullptr){return;}
	else
	{
		nodeQueue.push(tree.rootNode);
		while(!nodeQueue.empty())
		{
			temp=nodeQueue.front();
			printNode(temp);
			nodeQueue.pop();
			for(int i=0;i<temp->child.size();++i)
			{
				nodeQueue.push(temp->child.at(i));
			}
		}
	}
}

//funtion to Extend/Limit the rays to the Bounding Box(The first box in the QuadTree)
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
//Funtion to get the points lying inside the slab of spcified thickness/threshold and store it in an array 
template <typename Type1,typename Type2>
void thresh_Points(const Type1& Object1,const Type2& Object2,float threshold,std::vector<Type2>& Object3)
{
	if(CGAL::squared_distance(Object1,Object2)<=threshold)
	{
		Object3.push_back(Object2);
	}
}
/*
void thresh_Points(const Segment_2&  seg,const Point_2 point,const int& thresh,std::vector<Point_2>& NewThreshPoints)
{
	
	std::vector<Segment_2> threshSeg=bBox(seg,thresh);
	//extendSegments(box,threshSeg);
	threshSeg.erase(threshSeg.begin()+2);
	
	//std::cout<<"thresh_Points "<<seg<<" "<<point<<" "<<thresh<<" "<<threshSeg.size()<<std::endl;

	
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
		NewThreshPoints.push_back(point);
	}
	
	//std::cout<<"thresh_Points1 "<<threshPoints.size()<<std::endl;
}*/
//function to get the inside points for a given threshold
std::vector<Point_2> insidePoints(const QuadTree& tree,const Segment_2& edge,const int& thresh){
	std::vector<std::vector<Point_2> > tempPoints;
	tempPoints.reserve(5);
	std::queue<std::shared_ptr<Node>> nodeQueue;
	//std::unordered_map<Point_2,int> mapping;
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
	//std::cout<<th_Points.size()<<" Original"<<std::endl;
	/*for( std::unordered_map<Point_2,int>::iterator it=mapping.begin();it!=mapping.end();++it)
	{
		//std::cout<<(*it)->first<<std::endl;
		thresh_Points(edge,it->first,thresh,th1_Points);
	}
	std::cout<<th1_Points.size()<<" Dup"<<std::endl;*/
	return th_Points;
}

//Function to get the insidePoints for a given threshold
/*template <typename Type1> 
std::vector<Point_2> insidePoints(const QuadTree& tree,const Type1& edge,const int& thresh)
{
	std::vector<std::vector<Point_2> > tempPoints;
	std::queue<std::shared_ptr<Node>> nodeQueue;
	std::shared_ptr<Node> temp;
	Iso_rectangle_2 tempRect=bBox(tree.rootNode->rectangle,edge,thresh);
	if (tree.rootNode==nullptr){return;}
	else
	{
		nodeQueue.push(tree.rootNode);
		while(!nodeQueue.empty())
		{
			temp=nodeQueue.front();
			if(tree.is_leafNode(temp)){tempPoints.push_back(temp->insidePoints);}
			for(int i=0;i<temp->child.size();++i)
			{
				if(CGAL::do_intersect(tempRect,temp->child.at(i)->rectangle))
				{
					nodeQueue.push(temp->child.at(i));
				}
			}
			nodeQueue.pop();
		}
	}
	
	std::vector<Points_2> th_Points;
	for(int i=0;i<tempPoints.size();++i)
	{
		for(int j=0;j<tempPoints[i].size()++j)
		{
			thresh_Points(edge,tempPoints[i].at(j),thresh,th_Points);
		}
	}
	return th_Points;
	

}*/


//Function to get the minimum and Maximum Coordinates of the Point Set
void MinMaxCoor(std::vector<Point_2> &input,Iso_rectangle_2& box){
	//double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	std::vector<Point_2>::iterator it = input.begin();
	 Point_2 point;
	//string str;
	//while((std::getline(input,str))){
	while(it != input.end()){
        point = *it;
		//istringstream stream(str);
       // while((stream>>point)){
			if(minX>point.x()){minX=point.x();}
			if(minY>point.y()){minY=point.y();}
			if(maxX<point.x()){maxX=point.x();}
			if(maxY<point.y()){maxY=point.y();}
			++it;
        }
    //}
	//std::cout<<maxX<<" "<<minX<<" "<<maxY<<" "<<minY<<std::endl;
	maxX=maxX+100;maxY=maxY+100;minX=minX-100;minY=minY-100;
	Iso_rectangle_2 bbox(minX,minY,maxX,maxY);
	box=bbox;
}


//Funtion to Create Delaunay Triangulation from a file of points
void create_Delaunay(Delaunay& dt,std::vector<Point_2> &input)
{
	//std::istream_iterator <Point_2> begin(input);
   	//std::istream_iterator <Point_2> end;
   	dt.insert(input.begin(),input.end());
   	
}

//To assign ids to the Delaunay Triangulation
void Assign_ids(Delaunay& dt)
{
	int id = 0;
	Finite_vertices_iterator_2d vit;
	for (vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); vit++) {
		vit->id = id;
		id++;
	}
}

//To output Delaunay Triangulation in OFF format to view in MeshView/Geomview
void OFF_format(Delaunay& dt,std::ofstream &output)
{
	//get the number of vertices and faces

	int vertex_count = dt.number_of_vertices();
	int face_count = dt.number_of_faces();
	int correct_edge_count =0;
	Finite_vertices_iterator_2d vit;
	
	if(output.is_open())
	{
		output<<"OFF"<<std::endl<<vertex_count<<" "<<face_count<<" "<<"0"<<std::endl;
		for (vit = dt.finite_vertices_begin(); vit!= dt.finite_vertices_end(); vit++)
		{
			output<<vit->point()<<" 0 "<< std::endl;
		}	
	}

	for(fit f1 =dt.finite_faces_begin(); f1 != dt.finite_faces_end(); f1++)
	{
		vh first = (f1->vertex(0));
		unsigned int first_id = first->id;
		
		vh second = (f1->vertex(1));
		unsigned int second_id = second->id;

		vh third = (f1->vertex(2));
		unsigned int third_id = third->id;
	
		if(output.is_open())
		{
			output<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
			output<<f1->vertex(0)->point()<<" " <<f1->vertex(1)->point()<<" " <<f1->vertex(2)->point()<<std::endl;
			//std::cout<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
		}	
	}
  	output.close();
}
void OFF_formatInside(Delaunay& dt,std::ofstream &output)
{
	//get the number of vertices and faces

	int vertex_count = dt.number_of_vertices();
	int face_count = dt.number_of_faces();
	int correct_edge_count =0;
	Finite_vertices_iterator_2d vit;
	
	if(output.is_open())
	{
		output<<"OFF"<<std::endl<<vertex_count<<" "<<face_count<<" "<<"0"<<std::endl;
		for (vit = dt.finite_vertices_begin(); vit!= dt.finite_vertices_end(); vit++)
		{
			output<<vit->point()<<" 0 "<< std::endl;
		}	
	}
	
	fit f1 =dt.finite_faces_begin();
	for(; f1 != dt.finite_faces_end(); f1++)
	{
		if(f1->face_is_inside)
		{
			vh first = (f1->vertex(0));
			unsigned int first_id = first->id;
		
			vh second = (f1->vertex(1));
			unsigned int second_id = second->id;

			vh third = (f1->vertex(2));
			unsigned int third_id = third->id;
	
			if(output.is_open())
			{
				output<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
				//std::cout<<"3 "<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
			}	
		}
	}
  		output.close();
}
void Vect_format(std::vector<Point_2>& points,std::ofstream &output)
{
	if(output.is_open())
	{
		output<<"VECT"<<std::endl<<points.size()<<" "<<points.size()<<" "<<"3"<<std::endl<<std::endl;
		
		for(int i=0;i<points.size();i++)
		{
			output<<" 1 ";
		}
		output<<std::endl<<std::endl;
		for(int i=0;i<points.size();i++)
		{
			output<<" 1 ";
		}
		output<<std::endl<<std::endl;
		for (int i=0;i<points.size(); i++)
		{
			output<<points.at(i)<<" 0 "<< std::endl;
		}
		output<<std::endl;
		for (int i=0;i<points.size(); i++)
		{
			output<<" 1 "<<" 0 "<<" 0 "<<" 1 "<<std::endl;
		}		
	}
	output.close();
}

//Function to Output Delaunay Edges so as to view in Opengl
void outputDelaunayEdges(Delaunay dt,std::ofstream &output)
{
	for(fit f1 =dt.finite_faces_begin(); f1 != dt.finite_faces_end(); f1++)
	{
		vh first = (f1->vertex(0));
		vh second = (f1->vertex(1));
		vh third = (f1->vertex(2));
	
		if(output.is_open())
		{
			output<<first->point()<<" ";output<<second->point()<<std::endl;
			output<<second->point()<<" ";output<<third->point()<<std::endl;
			output<<third->point()<<" ";output<<first->point()<<std::endl;
		}		
	}
  	output.close();
} 

//Function to Output Delaunay Edges so as to view in Opengl
void outputInsideDelaunayEdges(Delaunay dt,std::ofstream &output,int& numInsideTri)
{
	fit f1 =dt.finite_faces_begin();

	for(; f1 != dt.finite_faces_end(); f1++)
	{
		if(f1->face_is_inside==true)
		{
			++numInsideTri;
			vh first = (f1->vertex(0));
			vh second = (f1->vertex(1));
			vh third = (f1->vertex(2));
	
			if(output.is_open())
			{
				output<<first->point()<<" ";output<<second->point()<<std::endl;
				output<<second->point()<<" ";output<<third->point()<<std::endl;
				output<<third->point()<<" ";output<<first->point()<<std::endl;
			}		
		}
	}
  	output.close();
} 

struct EdgeInfo
{
	fh face;
	int vertex_index;
	int index;
};

//Function to Create Voronoi Edges and store it as Segments and Rays in a vector
void create_Voronoi(Delaunay& dt,std::vector<Ray_2>& ray,std::vector<Segment_2>& seg,std::vector<EdgeInfo>& ray_edges,std::vector<EdgeInfo>& seg_edges)
{
	Edge_iterator eit =dt.edges_begin();
	
	for ( ; eit !=dt.edges_end();++eit) 
	{
        CGAL::Object o = dt.dual(eit);
		const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
        const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
        if (r) 
        {
        	ray.push_back(*r);

        	EdgeInfo info;
        	info.face=eit->first;
        	info.vertex_index=eit->second;
        	info.index=ray.size()-1;
        	ray_edges.push_back(info);
        }
        else if (s) 
        {
        	seg.push_back(*s);
        
        	EdgeInfo info;
        	info.face=eit->first;
        	info.vertex_index=eit->second;
        	info.index=seg.size()-1;
        	seg_edges.push_back(info);
        }
	}
}

//Funtion to get the points lying inside the slab of spcified thickness/threshold and store it in an array 
/*template <typename Type1,typename Type2>
void thresh_Points(const Type1& Object1,const Type2& Object2,float threshold,std::vector<Type2>& Object3)
{
	if(CGAL::squared_distance(Object1,Object2)<=threshold)
	{
		Object3.push_back(Object2);
	}
}*/

/*struct PointInfo
{
	Point_2 poi;
	int info;
};*/

//Function to get nearest and next_nearest neighbors using NNCrust and store it in a vector in the form {point,nearest,next_nearest}
template <typename Type>
void NNCrust(Delaunay& dt,std::vector<Point_2>& sample,std::vector<vector <Point_2> >& Neighbors,std::vector<Segment_2>& seg,Type ray,int threshold )
{
	
	
	Finite_vertices_iterator_2d vit;
	std::vector< vector <Point_2> > TestNeigh;
	//std::cout<<"Delaunay Vertices "<<dt.number_of_vertices()<<std::endl;
	//std::cout<<"Delaunay Faces "<<dt.number_of_faces()<<std::endl;
	//for(int i=0;i<sample.size();i++)
	for(vit = dt.finite_vertices_begin();vit!=dt.finite_vertices_end();vit++)
	{
		//std::cout<<sample.size()<<" Sample Size"<<std::endl;
		//std::cout<<"NNCrust "<<std::endl;
		//std::cout<<sample.at(i)<<" Sample Point"<<std::endl;
		bool notThresh_point=true;
		if(CGAL::squared_distance(ray,vit->point())<threshold)
		{
			notThresh_point=false;
		}
		else{notThresh_point=true;}
		//std::cout<<"NNCrust 1 "<<std::endl;
	if(notThresh_point==false)
	{
		//std::cout<<"NNCrust 2"<<std::endl;
		int count = 0;
		//store pair of vertex 
			
		//for each vertex find the Delaunay neighbors.
		std::vector<vh> neighbors;
		
		std::vector<Point_2> TestP;
		//PointInfo tempInfo;
		//tempInfo.poi=vit->point();
		//tempInfo.info=0;
		//TestP.push_back(tempInfo);
		TestP.push_back(vit->point());
		//get a face handle incident on vit
		fh f1 = vit->face();

		//find the index of vit in f1
		int index = f1->index(vit);
		int index1 = (index + 1)%3;
		int index2 = (index + 2)%3;
		//std::cout<<"NNCrust 3"<<std::endl;
		//neighbors.push_back(f1->vertex(index1));
		//neighbors.push_back(f1->vertex(index2));
		//count += 2;
		
		//start a loop to get all the neighbors
		/*do {
			fh f2 = f1->neighbor(index1);
			//std::cout<<"NNCrust 4"<<std::endl;
			//now find the index of vit in f2
			
			index = f2->index(vit);
			index1 = (index + 1)%3;
			index2 = (index + 2)%3;
			if(neighbors[count-1]->id == f2->vertex(index1)->id) {
				neighbors.push_back(f2->vertex(index2));
				++count;}
			else if (neighbors[count-1]->id == f2->vertex(index2)->id) {
					neighbors.push_back(f2->vertex(index1));
					++count;
					index1 = index2;}
			
			f1 = f2;
			}while(neighbors[0]->id != neighbors[count-1]->id);*/
		Vertex_circulator vcirculator=dt.incident_vertices(vit),done(vcirculator);
		do
		{
			if(!dt.is_infinite(vcirculator)){neighbors.push_back(vcirculator);}
		} while(++vcirculator != done);

		for(int i=0;i<neighbors.size();i++)
		{
			//std::cout<<"NNCrust 5"<<std::endl;
			//tempInfo.poi=neighbors[i]->point();
			//tempInfo.info=0;
			//TestP.push_back(tempInfo);
			TestP.push_back(neighbors[i]->point());
		}
		TestNeigh.push_back(TestP);
	}
	}
	
	
	for(int l=0;l<TestNeigh.size();++l)
	{
		int nearest=0;int next_nearest=0;
		double d_min = DBL_MAX,d_min1=DBL_MAX;
		for(int j=1;j<TestNeigh[l].size();++j)
		{
			double d= squared_distance(TestNeigh[l].at(0),TestNeigh[l].at(j));
			if(d<d_min)
			{
				d_min=d;
				nearest=j;
			}
		}
		for(int k=1;k<TestNeigh[l].size();++k)
		{
			Point p1 = TestNeigh[l].at(0);
			Point p2 = TestNeigh[l].at(nearest);
			Point p3 = TestNeigh[l].at(k);
			double dist = squared_distance(p1,p3); 
			
			if (((p2 != p3)&&(p1!=p3)) && (CGAL::angle(p2,p1,p3) != CGAL::ACUTE)) 
			{ 
				
				if (dist < d_min1) 
				{
					d_min1 = dist;
					next_nearest=k;
				}
				//std::cout<<TestNeigh[l].at(0)<<" "<<TestNeigh[l].at(k)<<" "<<next_nearest<<" "<<k<<" "<<dist<<" "<<d_min1<<" Distance "<<std::endl;
			}
		}
		std::vector<Point_2> TempNeighbors;
		
		//std::cout<<"NNCrust 6"<<std::endl;
		//std::cout<<TestNeigh.size()<<" Size of the TestNeigh"<<std::endl;
		//std::cout<<TestNeigh[l].size()<<" Size of the TestNeigh[i]"<<std::endl;
		//std::cout<<nearest<<" "<<next_nearest<<std::endl;
		
		//TestNeigh[i].at(0).info=TestNeigh[i].at(0).info+2;
		//TestNeigh[i].at(nearest).info=TestNeigh[i].at(nearest).info+1;
		//TestNeigh[i].at(next_nearest).info=TestNeigh[i].at(next_nearest).info+1;
		TempNeighbors.push_back(TestNeigh[l].at(0));
		//std::cout<<TestNeigh[l].at(0)<<" "<<TestNeigh[l].at(nearest)<<" "<<TestNeigh[l].at(next_nearest)<<std::endl;
		//std::cout<<"NNCrust 7"<<std::endl;
		TempNeighbors.push_back(TestNeigh[l].at(nearest));
		//std::cout<<"NNCrust 7.1"<<std::endl;
		
		TempNeighbors.push_back(TestNeigh[l].at(next_nearest));
		//if(TestNeigh[i].at(nearest).info<=2){TempNeighbors.push_back(TestNeigh[i].at(nearest).poi);}
		//if(TestNeigh[i].at(next_nearest).info<=2){TempNeighbors.push_back(TestNeigh[i].at(next_nearest).poi);}
		//std::cout<<"NNCrust 7.2"<<std::endl;
		Neighbors.push_back(TempNeighbors);			
	}
	//std::cout<<"NNCrust 8"<<std::endl;
	for(int p=0;p<Neighbors.size();p++)
	{
		Segment_2 segment1(Neighbors[p].at(0),Neighbors[p].at(1));
		Segment_2 segment2(Neighbors[p].at(0),Neighbors[p].at(2));
		seg.push_back(segment1);
		seg.push_back(segment2);
		//std::cout<<"NNCrust 8"<<std::endl;
		for(int q=0;q<seg.size()-1;q++)
		{
			for(int r=q;r<seg.size();r++)
			{
				if(((seg.at(q).source()==seg.at(r).target())&&(seg.at(q).target()==seg.at(r).source()))||((seg.at(r).target().x()-seg.at(r).source().x()<0.05)&&(seg.at(r).target().y()-seg.at(r).source().y()<0.05)))
				{
					seg.erase(seg.begin()+r);
				}
			}
		}
	}
		//std::cout<<"NNCrust 8"<<std::endl;
		
	//std::cout<<"Neighbouring Segment Size "<<seg.size()<<std::endl;
}

//Function to create the Segments
/*void createSegments(std::vector<vector <Point_2> >& Neighbors,std::vector<Segment_2>& seg)
{

	for(int i=0;i<Neighbors.size();i++)
	{
		Segment_2 segment1(Neighbors[i].at(0),Neighbors[i].at(1));
		Segment_2 segment2(Neighbors[i].at(0),Neighbors[i].at(2));
		seg.push_back(segment1);
		seg.push_back(segment2);
		for(int j=0;j<seg.size()-1;j++)
		{
			for(int k=j;k<seg.size();k++)
			{
				if((seg.at(j).source()==seg.at(k).target())&&(seg.at(j).target()==seg.at(k).source()))
				{
					seg.erase(seg.begin()+k);
				}
			}
		}
	}
}*/

//Function to check number of intersections
template <typename Type1,typename Type2>
int check_Intersection(Type1 rays,std::vector<Type2> seg)
{
	int count=0;
	bool on_theRay=false;
		for(int j=0;j<seg.size();j++)
		{
			if((seg.at(j).source().x()<DBL_MAX)||(seg.at(j).source().y()<DBL_MAX)||(seg.at(j).target().x()<DBL_MAX)||(seg.at(j).target().y()<DBL_MAX))
			{
				if(seg.at(j).source()!=seg.at(j).target())
				{
					if(CGAL::do_intersect(rays,seg.at(j)))
					{
						++count;
					}
					if((rays.has_on(seg.at(j).source()))||(rays.has_on(seg.at(j).target()))){on_theRay=true;}
				}
			}
		}
		if(on_theRay){--count;}
	return count;
}


std::ofstream mulInter;

//Function to get the farthest point
template <typename Type1,typename Type2,typename Type3>
Point_2 get_Farthest_Point(Type1& ray,std::vector<Type2>& seg,Type3& point)
{
	//std::cout<<"Error get Farthest Point"<<std::endl;
	//mulInter.open("MultipleIntersections.txt",std::ofstream::out | std::ofstream::trunc);
	Point_2 Point;
	//std::cout<<ray<<std::endl;
	//std::cout<<seg.size()<<std::endl;
	double d_max=0;
	//std::cout<<"get Farthest Point 1"<<std::endl;
	for(int i=0;i<seg.size();i++)
	{
		//std::cout<<"Error get Farthest Point 2"<<std::endl;
		//std::vector<Point_2> tempVec;
		Point_2 far_point;
		if((CGAL::do_intersect(ray,seg.at(i)))&&(seg.at(i).source()!=seg.at(i).target()))
		{
			
			const Point_2* tempPoint;
			//std::cout<<ray<<std::endl;
			//std::cout<<seg.at(i)<<std::endl;
			//std::cout<<"Error get Farthest Point 3"<<std::endl;
			//if(CGAL::x_equal(seg.at(i).source(),seg.at(i).target())){std::cout<<" Xequal segment is vertical"<<std::endl;}
			if(seg.at(i).is_vertical()){
				
				
				//std::cout<<" IsVertical segment is vertical"<<std::endl;
				double t =(seg.at(i).source().x()-ray.source().x())/ray.direction().dx();
				Point_2 pq(seg.at(i).source().x(),ray.source().y()+t*(ray.direction().dy()));
				far_point=pq;
				//std::cout<<" T "<<t<<std::endl;
				//if()
				
			}
			else{
			CGAL::Object obj = CGAL::intersection(ray,seg.at(i));
			tempPoint=CGAL::object_cast<Point_2>(&obj);
			if(tempPoint==NULL){std::cout<<"tempPoint is NULL "<<std::endl;std::cout<<seg.at(i)<<std::endl;}
			const Segment_2* tempSeg=CGAL::object_cast<Segment_2>(&obj);
			if(tempPoint){far_point=*tempPoint;//std::cout<<"It is a point"<<std::endl;
			}
			if(tempSeg){std::cout<<"It is a segment"<<std::endl;}
			/*if(mulInter.is_open())
			{
				std::cout<<"Error get Farthest Point 4"<<std::endl;
				//std::cout<<"open"<<std::endl;
				mulInter<<*tempPoint<<std::endl;
			}*/

			}
			//std::cout<<"Error get Farthest Point 5"<<std::endl;
			//std::cout<<*tempPoint<<std::endl;
			//std::cout<<"Error get Farthest Point 6"<<std::endl;
			
				//std::cout<<*tempPoint<<" TempPoint"<<std::endl;
				//std::cout<<"Error get Farthest Point 7"<<std::endl;
				double dist=CGAL::squared_distance(point,far_point);
				//std::cout<<"Error get Farthest Point 8"<<std::endl;
				if(dist>d_max)
				{
					//std::cout<<"Error get Farthest Point 9"<<std::endl;
					Point = far_point;
					d_max=dist;
				}
			
		}
		else{//std::cout<<"It is not intersecting"<<std::endl;
		}
		/*if(mulInter.is_open())
		{
		for(int i=0;i<tempVec.size();i++)
		{
			std::cout<<"open"<<std::endl;
			mulInter<<tempVec.at(i)<<std::endl;
		}
		}*/
		//tempVec.clear();

	}
	//mulInter.close();
	return Point;

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

bool isSkinnyTriangle(fh faceItr)
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

	//double result = 2.0;
	//if r/l ratio is greater than sqrt(2.0)
	/*
	if(circumradius/s_distance >= 0.2 && circumradius/s_distance < 0.5)				// SKINNY TRIANGLES
		{
			skinny_bound_zero++;
			//answer = false;
			//std::cout<<"it is a skinny triangle"<<std::endl;
		}
	
	if(circumradius/s_distance >= 0.5 && circumradius/s_distance < 1.0)				// SKINNY TRIANGLES
		{
			skinny_bound_one++;
			//answer = true;
			//std::cout<<"it is a skinny triangle"<<std::endl;
		}
	if(circumradius/s_distance >= 1.5 && circumradius/s_distance <= 2)				// SKINNY TRIANGLES
		{
			skinny_bound_two++;
			//answer = true;
			//std::cout<<"it is a skinny triangle"<<std::endl;
		}*/
	if(circumradius/s_distance > 2)				// SKINNY TRIANGLES
		{
			//skinny_bound_three++;
			answer = true;
			//std::cout<<"it is a skinny triangle"<<std::endl;
		}
	
	return answer;
}
bool is_circumcenter_inside(Point_1 p1, Point_1 p2, Point_1 p3, Point_1 q)
{
	//if orientation(p1,p2,q) == CGAL::COLLINEAR || .... return 1 

	if(CGAL::collinear(p1, p2, q) || CGAL::collinear(p2, p3, q) || CGAL::collinear(p3, p1, q))
	{	
		return 1;
	}
	
	// if orientation(p1,p2,q) == orientation(p2,p3,q) == orientation (p3, p1, q)
	//then retun 1 else return 0

	else if((CGAL::orientation(p1, p2, q) == CGAL::orientation(p2, p3, q)) && (CGAL::orientation(p2, p3, q) == CGAL::orientation(p3, p1, q)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

fh circumcenter_position(fh faceItrs, bool &circumcenter_is_outside,Point& circum_center )
{		
		int not_to_check = -1;
		circumcenter_is_outside = true;		
		fh next_face;
		bool move_ahead = true;

		vh first = (faceItrs->vertex(0));
		//unsigned int first_id = first->id; 

		vh second = (faceItrs->vertex(1));
		//unsigned int second_id = second->id;

		vh third = (faceItrs->vertex(2));
		//unsigned int third_id = third->id;	
			
		//Find centroid
		Point centroid = CGAL::centroid(first->point(), second->point(), third->point());
		Point c = CGAL::circumcenter(first->point(), second->point(), third->point());

		Point_1 p1(c.x(), c.y());
		Point_1 p2(centroid.x(), centroid.y());
		// Line between centroid and circumcenter.
		line_2d l(p1, p2);	
				
		//int debug_counter = 0;
		while(move_ahead)
		{
		//	std::cout<<" Inside loop "<<std::endl;					
			Point_1 p_first(first->point().x(), first->point().y());
			Point_1 p_second(second->point().x(), second->point().y());
			Point_1 p_third(third->point().x(), third->point().y());
			
			line_2d line_1(p_first, p_second);  // Line between first and second point of triangle
			line_2d line_2(p_second, p_third);	// Line between second and third point of triangle
			line_2d line_3(p_third, p_first);	// Line between third and first point of triangle

			//std::cout<<first_id<<' '<<second_id<<' '<<third_id<<std::endl;
			//std::cout<<"circumcenter is : "<<p1<<std::endl;
			
			// FIRST CHECK IF CIRCUMCENTER IS INSIDE THIS TRIANGLE
			if(is_circumcenter_inside(p_first, p_second, p_third, p1))
			{
				//debug
				//std::cout<<"cirumcenter is inside"<<std::endl;

				if(!faceItrs->face_is_inside)		//face is outside
				{
					circumcenter_is_outside = true;
					//return faceItrs;
				}
				else
				{
					circumcenter_is_outside = false;
					//return faceItrs;
				}
				move_ahead = false;
				break;
			}
			else 
			{
				//debug_counter++;

				//std::cout<<"cirumcenter is not inside and not_to_check = "<<not_to_check<<std::endl;
				if(not_to_check==-1 || not_to_check != 2){
					//std::cout<<" 1. "<<std::endl;
					if(CGAL::do_intersect(l, line_1) == true)
					{
						//std::cout<<" failure 1"<<std::endl;
					//if(not_to_check==-1 || not_to_check == 2)
					//{ 
						next_face = faceItrs->neighbor(2);
						//std::cout<<"its not checking 2"<<std::endl;
					}
				}
				if (not_to_check==-1 || not_to_check != 0){
					//std::cout<<" 2. "<<std::endl;
					if(CGAL::do_intersect(l, line_2) == true )
					{
						//if(not_to_check == -1 || not_to_check == 0)
						//{ 
						next_face = faceItrs->neighbor(0);
						//std::cout<<"its not checking 0"<<std::endl;
					}
				}
				if (not_to_check==-1 || not_to_check != 1){
					//std::cout<<" 3. "<<std::endl;
					if(CGAL::do_intersect(l, line_3) == true )
					{
						//if(not_to_check == -1 || not_to_check == 1)
						//{
						next_face = faceItrs->neighbor(1);
						//std::cout<<"its not checking 1"<<std::endl;
					}
				}
			
				not_to_check = next_face->index(faceItrs);
				assert(next_face->neighbor(not_to_check) == faceItrs);
				faceItrs = next_face;
				//std::cout<<"got the nxt face"<<std::endl;
				
			}
			first = (faceItrs->vertex(0));
			//first_id = first->id; 

			second = (faceItrs->vertex(1));
			//second_id = second->id;

			third = (faceItrs->vertex(2));
			//third_id = third->id;	
			
			//if(debug_counter == 3)
				//break;
		}
		circum_center=c;
		//circumcenter_is_outside = !circumcenter_is_outside; 
		return faceItrs;
}
int main()
{
	int stop;
	std::clock_t t1,t2;
	t1=clock();
	std::vector<Point_2> OriginalSample,RandomSample;
	//std::srand(time(NULL));
	
	//Inputs and Outputs from/to various files
	std::ifstream input("Two_Quarter_76004.txt");
	//std::ofstream outputRandomSample("RandomSample.txt");
	
	int num_of_points=0;
	std::string data;
	while(getline(input,data))
	{
		Point_2 original;
		std::istringstream stream(data);
		while(stream>>original)
		{	
			OriginalSample.push_back(original);
			++num_of_points;
		}
	}
		
	//std::ifstream input1("Points_xy_500.txt");
	Iso_rectangle_2	boundingBox;
	MinMaxCoor(OriginalSample,boundingBox);
	QuadTree tree(boundingBox);
	
	tree.rootNode->insidePoints=OriginalSample;
	generate_QuadTree(tree,tree.rootNode);
	//printTree(tree);
	//std::cout<<"1"<<std::endl;
	/*std::cout<<"Input 1 to random and 0 not random"<<std::endl;
	int con_input;
	std::cin>>con_input;

	//std::cout<<num_of_points<<" Num of Points"<<std::endl;
	
	if(con_input==1)
	{
	//Taking some random sample from the original sample
	for(int i=0;i<4;i++)
	{
		int n=std::rand()%(num_of_points-1);
		RandomSample.push_back(OriginalSample.at(n));
		if(outputRandomSample.is_open()){outputRandomSample<<OriginalSample.at(n)<<std::endl;}
		//if(outputRandomSampleIndices.is_open()){outputRandomSampleIndices<<n<<std::endl;}
	}
	}*/
	Point_2 p1,p2,p3,p4;
	p1 = Point_2(0,5000);
	p2 = Point_2(0,10000);
	p3 = Point_2(5000,0);
	p4 = Point_2(10000,0);

	RandomSample.push_back(p1);
	RandomSample.push_back(p2);
	RandomSample.push_back(p3);
	RandomSample.push_back(p4);
	//Creating Delaunay Triangulation of the points in the inputRandomSample
	Delaunay dt_sample;
	create_Delaunay(dt_sample,RandomSample);			
	
	bool iterate=true;
	while(iterate)
	 {  //1
		 std::ofstream HomoEdges;
		HomoEdges.open("HomomorphicalEdges.txt",std::ofstream::out | std::ofstream::trunc);
		//std::cout<<"Input 1 to continue iteration"<<std::endl;
		//int console_input;
		//std::cin>>console_input;

		 //Ofstream for ouputPoints i.e the Threshold Points
		 std::ofstream outputPoints;
		
		 int number_of_skinny_triangle = 0;
		 //int skinny_bound_zero = 0;
		 //int skinny_bound_one = 0;
		 //int skinny_bound_two = 0;
		 int skinny_bound_three = 0;
		// int skinny_bound_is_two = 0;

		
			//Declaring boolean values to check whether Rays and Segments have only one intersection
			bool rays_ok=true;
			bool segs_ok=true;
			//std::cout<<"iterate 2"<<std::endl;
			Edge_iterator eit =dt_sample.edges_begin();
	
			for ( ; eit !=dt_sample.edges_end();eit++) 
			 {  
        		CGAL::Object o = dt_sample.dual(eit);
				//std::cout<<"iterate Crucial 1"<<std::endl;
				const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
				//std::cout<<"iterate Crucial 2"<<std::endl;
        		const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
        		//std::cout<<"iterate 3"<<std::endl;
				if (r)
        		{  //4
					int num_of_intersections=0;
        			std::vector<Point_2> ThreshPoints, NewThreshPoints;
					std::vector<vector <Point_2> > Neighbors;
					std::vector<Segment_2> Neighbor_Segments;
					Segment_2 temp;
					if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
						temp=convToSeg(tree.rootNode->rectangle,*r);
						ThreshPoints = insidePoints(tree, temp, 60);
					}
				/*	for(int j=0;j<OriginalSample.size();j++)
					{						
						thresh_Points(*r,OriginalSample.at(j),500,ThreshPoints);
					}
					*/
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

					std::cout<<ThreshPoints.size()<<" Thresh Points"<<std::endl;
					
					Delaunay dt_thresh;
					create_Delaunay(dt_thresh,ThreshPoints);		
					Assign_ids(dt_thresh);
										
					NewThreshPoints = insidePoints(tree, temp, 40);
					std::cout<<NewThreshPoints.size()<<" New Thresh Points"<<std::endl;

					for(int j=0;j<NewThreshPoints.size();++j)
					{
						for(int k=j;k<NewThreshPoints.size();++k)
						{
							if(j==k){continue;}
							else
							{
								if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
								{
									//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
									NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
								}
								
							}
						}
					}
					
					if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
					{						
						NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*r,40);
						num_of_intersections=check_Intersection(*r,Neighbor_Segments);
					}
					std::cout<<num_of_intersections<<" Number of Intersections"<<std::endl;
					
					if(num_of_intersections>1)
					 {  
						iterate=true;
						//Rays have multiple Intersections
						rays_ok=false;	
						Point_2 Far_Point=get_Farthest_Point(*r,Neighbor_Segments,eit->first->vertex((eit->second+1)%3)->point());
				
						vh tempVhandle=dt_sample.insert(Far_Point);
						++points_inserted;
						deFace(dt_sample,tempVhandle);

						//if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;}
												
						break;
					 }  //5
					if(num_of_intersections==1)
					{
						eit->first->correct_segments[eit->second]=true;
						HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
												
						fh opp_face = eit->first->neighbor(eit->second);
						int opp_index = opp_face->index(eit->first);
						opp_face->correct_segments[opp_index] = true;
					}										
				 }  //4
        
        		 if (s) 
        		 {	//4
        		
					std::vector<Point_2> ThreshPoints, NewThreshPoints;
					std::vector<vector <Point_2> > Neighbors;
					std::vector<Segment_2> Neighbor_Segments;	
										
					ThreshPoints = insidePoints(tree, *s, 60);
					

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

					std::cout<<ThreshPoints.size()<<"  Thresh Points"<<std::endl;	
					Delaunay dt_thresh;
					create_Delaunay(dt_thresh,ThreshPoints);		
					Assign_ids(dt_thresh);
					
					NewThreshPoints = insidePoints(tree, *s, 40);
					
					std::cout<<NewThreshPoints.size()<<"  NewThresh Points"<<std::endl;	

					int num_of_intersections=0;
					
					for(int j=0;j<NewThreshPoints.size();++j)
					{
						for(int k=j;k<NewThreshPoints.size();++k)
						{
							if(j==k){continue;}
							else
							{
								if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
								{
									//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
									NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
								}
								
							}
						}
					}
					
					if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
					{
						NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*s,40);
						num_of_intersections=check_Intersection(*s,Neighbor_Segments);
					}
					std::cout<<num_of_intersections<<" Number of Intersections"<<std::endl;
				
					if(num_of_intersections>1)
					 {  //5
						iterate=true;
						//Segs have multiple Intersections
						segs_ok=false;	
						Point_2 Far_Point=get_Farthest_Point(*s,Neighbor_Segments,eit->first->vertex((eit->second+1)%3)->point());
				
						vh tempVhandle=dt_sample.insert(Far_Point);
						++points_inserted;
						deFace(dt_sample,tempVhandle);

						//if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;}
						break;
					 }  //5
					if(num_of_intersections==1)
					{
					
						eit->first->correct_segments[eit->second]=true;
						HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
						
						fh opp_face = eit->first->neighbor(eit->second);
						int opp_index = opp_face->index(eit->first);
						opp_face->correct_segments[opp_index] = true;
					}					
				}  //4
				//Assign_ids(dt_sample);
			}  //3
			
			
			if((rays_ok!=true)||(segs_ok!=true))
			{
				all_fit a_fit;
				a_fit=dt_sample.all_faces_begin();
				int triangle_i = 1;
				for ( ; a_fit !=dt_sample.all_faces_end();++a_fit) 
				{
					for(int i = 0; i < 3; i++)
					{
					a_fit->correct_segments[i] = false;
					}
					triangle_i++;
				}	
			}
			
			if((rays_ok==true)&&(segs_ok==true))
			{  //3
				
				iterate=false;
				eit=dt_sample.edges_begin();
				for ( ; eit !=dt_sample.edges_end();++eit) 
				{  //4
						if(eit->first->correct_segments[eit->second]==true)
						{  //5
							//HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
						for(int i = 1; i < 3; i++)
						{	//6
							int count=0;	
							vh vh_center = eit->first->vertex((eit->second+i)%3);		
							Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);				
							do
							{
								if(circulator->first->correct_segments[circulator->second]==true)
								{
									++count;
								}
							
							}while(++circulator!=done);
							
							if((count>2))
							 {  //7
								iterate=true;
								Point_2 Farthest_Point =vh_center->point();
								Edge_circulator circulator1=dt_sample.incident_edges(vh_center),done(circulator1);				

								do
								 {  //8
									if(circulator1->first->correct_segments[circulator1->second]==true)
									 {  //9

										CGAL::Object o = dt_sample.dual(circulator1);
										const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
										const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
    		  							Point_2 Far_Point;
    
        								if(r)
        								 {  //10
        									std::vector<Point_2> ThreshPoints,NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;
        									
											Segment_2 temp;
											if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
												temp=convToSeg(tree.rootNode->rectangle,*r);
												ThreshPoints = insidePoints(tree, temp, 60);
											}

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

											std::cout<<ThreshPoints.size()<<"  Thresh Points"<<std::endl;	
											Delaunay dt_thresh;
											create_Delaunay(dt_thresh,ThreshPoints);		
											Assign_ids(dt_thresh);
											
											std::vector<Segment_2> Neighbor_Segments;	
				
												NewThreshPoints = insidePoints(tree, temp, 40);
											

											for(int j=0;j<NewThreshPoints.size();++j)
											{
												for(int k=j;k<NewThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
														}
								
													}
												}
											}
					
											std::cout<<NewThreshPoints.size()<<"  NewThresh Points"<<std::endl;	
											if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
											NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*r,40);
											Far_Point=get_Farthest_Point(*r,Neighbor_Segments,vh_center->point());
											//if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;}

        								 }  //10
        								if(s)
        								 {  //10
        									std::vector<Point_2> ThreshPoints,NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;
        									
												ThreshPoints = insidePoints(tree, *s, 60);
											
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

											std::cout<<ThreshPoints.size()<<"  Thresh Points"<<std::endl;	
											Delaunay dt_thresh;
											create_Delaunay(dt_thresh,ThreshPoints);		
											Assign_ids(dt_thresh);
											
											std::vector<Segment_2> Neighbor_Segments;	
															
											NewThreshPoints = insidePoints(tree, *s, 40);
											
											for(int j=0;j<NewThreshPoints.size();++j)
											{
												for(int k=j;k<NewThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
														}
								
													}
												}
											}
					
											
											std::cout<<NewThreshPoints.size()<<"  NewThresh Points"<<std::endl;	
											
											if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
											NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*s,40);
											
											Far_Point=get_Farthest_Point(*s,Neighbor_Segments,vh_center->point());
										//	if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;}
											
        								 }  //10
        								
        								if((CGAL::squared_distance(vh_center->point(),Far_Point)) > (CGAL::squared_distance(vh_center->point(),Farthest_Point)))
        								{
        									Farthest_Point=Far_Point;
        								}
        							 }  //9
							
								}while(++circulator1!=done); //8 
								vh tempVhandle=dt_sample.insert(Farthest_Point);
								++points_inserted;
								deFace(dt_sample,tempVhandle);
															
								all_fit a_fit1;	
								a_fit1=dt_sample.all_faces_begin();
								int triangle_i1 = 1;
								for ( ; a_fit1 !=dt_sample.all_faces_end();++a_fit1) 
								{
									for(int i1 = 0; i1 < 3; i1++)
									{
									a_fit1->correct_segments[i1] = false;
									}
									triangle_i1++;
								}						
								break;
							 }  //7
						}  //6 
					}   //5 
					if(iterate==true){break;}
					
				 }  //4

				if(iterate==false)
				 {  //4
					std::cout<<"Geometric/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////"<<std::endl;
					
					eit=dt_sample.edges_begin();
					for ( ; eit !=dt_sample.edges_end();++eit) 
					 {  //5
						if(eit->first->correct_segments[eit->second]==true)
						 {  //6
							for(int i = 1; i < 3; i++)
							 {	//7
								
								int count=0;
								vh vh_center = eit->first->vertex((eit->second+i)%3);		
								vh vh_adj,vh_adj1;
								std::vector<Edge_circulator> edge_cir;

								Edge_circulator circulator_edge;
								Edge_circulator circulator_edge1;
								
								Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);				
							
								do
								{ //8
									if(circulator->first->correct_segments[circulator->second]==true)
									 {  //9
										if(count<1){circulator_edge=circulator;}
										if(count==1){circulator_edge1=circulator;}
										if(vh_center==circulator->first->vertex((circulator->second+1)%3))
										{
											if(count<1){vh_adj=circulator->first->vertex((circulator->second+2)%3);}
											if(count==1){vh_adj1=circulator->first->vertex((circulator->second+2)%3);}
										}
										if(vh_center==circulator->first->vertex((circulator->second+2)%3))
										{
											if(count<1){vh_adj=circulator->first->vertex((circulator->second+1)%3);}
											if(count==1){vh_adj1=circulator->first->vertex((circulator->second+1)%3);}
										}
										edge_cir.push_back(circulator);
										++count;
									 }  //9
								 }while(++circulator!=done); //8
								if((CGAL::squared_distance(vh_center->point(), vh_adj->point()) < 3) && (CGAL::squared_distance(vh_center->point(), vh_adj1->point()) < 3))
								 {  
									edge_cir.clear();
									break;
								 }  
								
								if(vh_center==nullptr){std::cout<<"it is null"<<std::endl;}
								if(vh_adj==nullptr){std::cout<<"it is null 1"<<std::endl;}
								if(vh_adj1==nullptr){std::cout<<"it is null 2"<<std::endl;}
								double vec1=(vh_center->point().x()-vh_adj->point().x())*(vh_center->point().x()-vh_adj1->point().x());
								double vec2=(vh_center->point().y()-vh_adj->point().y())*(vh_center->point().y()-vh_adj1->point().y());

								double denom=(std::sqrt(((vh_center->point().x()-vh_adj->point().x())*(vh_center->point().x()-vh_adj->point().x()))+(vh_center->point().y()-vh_adj->point().y())*(vh_center->point().y()-vh_adj->point().y())))*(std::sqrt(((vh_center->point().x()-vh_adj1->point().x())*(vh_center->point().x()-vh_adj1->point().x()))+(vh_center->point().y()-vh_adj1->point().y())*(vh_center->point().y()-vh_adj1->point().y())));
								double ang;
								if(denom!=0){ang=std::acos((vec1+vec2)/denom);}
								if(ang<(160*3.14)/180)
								{  //8 
									
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
        							 {  //9
										std::vector<Point_2> ThreshPoints,NewThreshPoints;
        								std::vector<vector <Point_2> > Neighbors;
										std::vector<Segment_2> Neighbor_Segments;	
        									
										Segment_2 temp;
										if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
											temp=convToSeg(tree.rootNode->rectangle,*r);
											ThreshPoints = insidePoints(tree, temp, 60);
										}
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

										std::cout<<ThreshPoints.size()<<"  Thresh Points"<<std::endl;		
										Delaunay dt_thresh;
										create_Delaunay(dt_thresh,ThreshPoints);		
										Assign_ids(dt_thresh);
																				
										NewThreshPoints = insidePoints(tree, temp, 40);
											
										

										for(int j=0;j<NewThreshPoints.size();++j)
										{
											for(int k=j;k<NewThreshPoints.size();++k)
											{
												if(j==k){continue;}
												else
												{
													if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
													{
														//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
														NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
													}
								
												}
											}
										}
										std::cout<<NewThreshPoints.size()<<"  NewThresh Points"<<std::endl;	
										if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
										{
											NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*r,40);
										}
										Far_Point=get_Farthest_Point(*r,Neighbor_Segments,vh_center->point());
											
        							 }  //9
        							if(s)
        							{  //9 
        								std::vector<Point_2> ThreshPoints,NewThreshPoints;
        									std::vector<vector <Point_2> > Neighbors;
											std::vector<Segment_2> Neighbor_Segments;	
        																			
											ThreshPoints = insidePoints(tree, *s, 60);
											
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

											std::cout<<ThreshPoints.size()<<"  Thresh Points"<<std::endl;	
											Delaunay dt_thresh;
											create_Delaunay(dt_thresh,ThreshPoints);		
											Assign_ids(dt_thresh);
																						
											NewThreshPoints = insidePoints(tree, *s, 40);
											
											
											for(int j=0;j<NewThreshPoints.size();++j)
											{
												for(int k=j;k<NewThreshPoints.size();++k)
												{
													if(j==k){continue;}
													else
													{
														if(NewThreshPoints.at(j)==NewThreshPoints.at(k))
														{
															//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
															NewThreshPoints.erase(NewThreshPoints.begin()+k);	--k;
														}
								
													}
												}
											}
											std::cout<<NewThreshPoints.size()<<"  NewThresh Points"<<std::endl;	
											if(ThreshPoints.size()>2 && NewThreshPoints.size() > 0)
											{
												NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*s,40);
											}
											Far_Point=get_Farthest_Point(*s,Neighbor_Segments,vh_center->point());
											
        							 }	//9
									
									
									vh tempVhandle=dt_sample.insert(Far_Point);
									++points_inserted;
									deFace(dt_sample,tempVhandle);

									//if(outputRandomSample.is_open()){outputRandomSample<<Far_Point<<std::endl;}
									
									
									all_fit a_fit1;	
									a_fit1=dt_sample.all_faces_begin();
									int triangle_i1 = 1;
									for ( ; a_fit1 !=dt_sample.all_faces_end();++a_fit1) 
									{
										for(int i2 = 0; i2 < 3; i2++){
										a_fit1->correct_segments[i2] = false;
										}
										triangle_i1++;
									}
									
									iterate=true;
									break;
								}  //8 
								
								edge_cir.clear();
								
							 }  //7
							
						 }  //6
						if(iterate==true){break;}
						
					 }  //5
					
				 }  //4
				
			 }  //3
			
			 if(iterate==false)
			 {
			 std::cout<<"Mesh Refinement /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////"<<std::endl;
			 int insideTri=0;
			 int inside_faces_count = 0;
			 int vertex_count=0;
			 fh f_handle = dt_sample.incident_faces(dt_sample.infinite_vertex());
			 std::stack<fh> fh_stack;
			 fh_stack.push(f_handle);
			 do
			 {
				f_handle = fh_stack.top();	
				fh_stack.pop();
				
				if(f_handle->face_is_inside == true)
				{
					
					f_handle->face_is_inside = false;
					for(int i = 0; i < 3; i++)
					{
						if(f_handle->correct_segments[i]==false)
						{							
							fh_stack.push(f_handle->neighbor(i));
						}
						
					}
				}
				
			}while(!fh_stack.empty());
			Finite_faces_iterator_2d face_iterator;
			for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); face_iterator++) 
			{
				if(face_iterator->face_is_inside == true)
				{
					inside_faces_count++;
				}
			}
			
			int debug_counter= 0;	
			
			for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); face_iterator++)
			{
				Point circum_center;
				bool skinny_triangle = false;
				bool Circumcenter_is_outside = false;
				if(face_iterator->face_is_inside == true)
				{				
					skinny_triangle = isSkinnyTriangle(face_iterator);
					if(skinny_triangle)
					{
						number_of_skinny_triangle++;
						int skinny;
						fh containing_face = circumcenter_position(face_iterator, Circumcenter_is_outside,circum_center);
					
						if (!Circumcenter_is_outside)
						{
							vh tempVhandle=dt_sample.insert(circum_center);
							++points_inserted;
							deFace(dt_sample,tempVhandle);
						//	if(outputRandomSample.is_open()){outputRandomSample<<circum_center<<std::endl;}
							
							
							all_fit a_fit1;	
							a_fit1=dt_sample.all_faces_begin();
							int triangle_i1 = 1;
							for ( ; a_fit1 !=dt_sample.all_faces_end();++a_fit1) 
							{
								for(int i2 = 0; i2 < 3; i2++){
									a_fit1->correct_segments[i2] = false;
								}
								a_fit1->face_is_inside=true;
								triangle_i1++;
							}
							iterate=true;
							break;
						}
					}
					++insideTri;
				}
			}
			std::cout<<"Inside Triangles  "<<insideTri<<std::endl;	
		}
		if(iterate==false){
			std::cout<<"number of skinny triangles "<<number_of_skinny_triangle<<std::endl;
		}
	}  //1 		

	//std::ofstream outputDelaunayInside;
			
	//int insideTri=0;
	//outputInsideDelaunayEdges(dt_sample,outputDelaunayInside,insideTri);
	
	std::cout<<"Number of inserted points are = "<< points_inserted<<std::endl;
	t2=clock();
	float timeSec=((float)(t2)-(float)(t1))/CLOCKS_PER_SEC;
	std::cout<<"Time Taken for Execution: "<<timeSec<<std::endl;
	
	int k;
	std::cin>>k;
	return 0;
}
