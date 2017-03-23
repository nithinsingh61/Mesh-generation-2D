

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
#include <map>
#include <unordered_map>
#include <iostream>
#include <memory>

#define OT 5
#define IT 3

//End of Header Files
typedef my_K::Intersect_2									Intersect_2;

typedef CGAL::Point_2<Rep>									Point_1;
typedef CGAL::Line_2<Rep>									Line_2;
typedef Delaunay::Edge_iterator                     		Edge_iterator;
typedef Delaunay::Edge_circulator                     		Edge_circulator;
typedef Delaunay::Vertex_iterator							Vertex_iterator;
typedef my_K::Point_2		                     		    Point_2;
typedef my_K::Vector_2		                     		    Vector_2;
typedef my_K::Segment_2		                     		    Segment_2;
typedef my_K::Ray_2										    Ray_2;
//typedef my_K::Line_2										Line_2;
typedef my_K::Direction_2									Direction_2;
typedef my_K::Iso_rectangle_2								Iso_rectangle_2;
typedef Delaunay::Edge										Edge;


//typedef pair<vh, vh>										edge_vh;
//End of Typedef's
	
//int points_inserted = 0;
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

	boost::shared_ptr<Node> parent;
	std::vector<boost::shared_ptr<Node> > child;

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
	bool is_leafNode(const boost::shared_ptr<Node>& node);
	void spilt(boost::shared_ptr<Node>& node);
	void insertPoints(boost::shared_ptr<Node>& node);
	int checkNumPoints(const Node& node);

	//Point_2 leftDown,rightUp;
	boost::shared_ptr<Node> rootNode;
	boost::shared_ptr<Node> leafNode;
};

QuadTree::QuadTree(){}
//QuadTree::~QuadTree(){}
QuadTree::QuadTree(const Point_2& bottom,const Point_2& top){rootNode->maxCoor=top;rootNode->minCoor= Point_2(bottom);}
QuadTree::QuadTree(const Iso_rectangle_2& box):rootNode(new Node(box)){}


bool QuadTree::is_leafNode(const boost::shared_ptr<Node>& node)
{
	//std::cout.precision(17);
	return node->child.size()==0;
}

void QuadTree::spilt(boost::shared_ptr<Node>& node)
{
	//std::cout.precision(17);
	//CounterClockWise arrangement of Children startting from left bottom
	Point_2 midPoint((node->minCoor.x()+node->maxCoor.x())/2,(node->minCoor.y()+node->maxCoor.y())/2);
	
	Iso_rectangle_2 r1(node->minCoor.x(),node->minCoor.y(),midPoint.x(),midPoint.y());
	Iso_rectangle_2 r2(midPoint.x(),node->minCoor.y(),node->maxCoor.x(),midPoint.y());
	Iso_rectangle_2 r3(midPoint.x(),midPoint.y(),node->maxCoor.x(),node->maxCoor.y());
	Iso_rectangle_2 r4(node->minCoor.x(),midPoint.y(),midPoint.x(),node->maxCoor.y());
	
	node->child.push_back(boost::shared_ptr<Node>(new Node(r1)));
	node->child.push_back(boost::shared_ptr<Node>(new Node(r2)));
	node->child.push_back(boost::shared_ptr<Node>(new Node(r3)));
	node->child.push_back(boost::shared_ptr<Node>(new Node(r4)));
	
	for(int i =0;i<node->child.size();++i){node->child.at(i)->parent=node;}
}

void QuadTree::insertPoints(boost::shared_ptr<Node>& node)
{
	//std::cout.precision(17);
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
	//std::cout.precision(17);
	return node.insidePoints.size();
}

boost::shared_ptr<Node> generate_QuadTree(QuadTree& tree,boost::shared_ptr<Node>& node)
{
	//std::cout.precision(17);
	boost::shared_ptr<Node> temp;
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

void printNode(boost::shared_ptr<Node>& node)
{
	//std::cout.precision(17);
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

		//std::cout<<node->child.size()<<"Size////////////////////"<<std::endl;
		//std::cout<<node->insidePoints.size()<<"//////////////////////Inside Points"<<std::endl;
	}
	else return;
}

void printTree(const QuadTree& tree)
{
	//std::cout.precision(17);
	std::queue<boost::shared_ptr<Node>> nodeQueue;
	boost::shared_ptr<Node> temp;
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

//Function to get the minimum and Maximum Coordinates of the Point Set
void MinMaxCoor(std::vector<Point_2> &input,Iso_rectangle_2& box){
	//double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	double maxX=0,minX=DBL_MAX,maxY=0,minY=DBL_MAX;
	//std::cout.precision(17);
	std::vector<Point_2>::iterator it = input.begin();
	 Point_2 point;
	//string str;
	//while((std::getline(input,str))){
	while(it != input.end()){
        point = *it;
		//istringstream stream(str);
       // while((stream>>point)){
			if(minX>point.x()){minX=(CGAL::to_double(point.x()));}
			if(minY>point.y()){minY=CGAL::to_double(point.y());}
			if(maxX<point.x()){maxX=CGAL::to_double(point.x());}
			if(maxY<point.y()){maxY=CGAL::to_double(point.y());}
			++it;
        }
    //}
	//std::cout<<maxX<<" "<<minX<<" "<<maxY<<" "<<minY<<std::endl;
	maxX=maxX+100;maxY=maxY+100;minX=minX-100;minY=minY-100;
	Iso_rectangle_2 bbox(minX,minY,maxX,maxY);
	box=bbox;
}

//funtion to Extend/Limit the rays to the Bounding Box(The first box in the QuadTree)
Segment_2 convToSeg(const Iso_rectangle_2& box,const Ray_2& ray){
	//std::cout.precision(17);
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
std::vector<Segment_2> bBox(const Segment_2& edge,const double& thresh)
{
	//std::cout.precision(17);
	double dist=thresh;
	dist=std::sqrt(dist);
	//std::cout<<"inside bBox"<<std::endl;
	Segment_2 tempSeg(Point_2(0,0),Point_2(0,0));
	
	//std::cout<<edge.direction().dx()<<"  "<<edge.direction().dy()<<std::endl;
		
	Direction_2 dir;
	dir = edge.direction();
	//std::cout<<"inside bBox"<<std::endl;
	//std::cout<<"dir.dx	"<<dir.dx()<<std::endl;
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
		double slope=CGAL::to_double(dir.dy()/dir.dx());
		
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
void thresh_Points(const Type1& Object1,const Type2& Object2,const double& threshold,std::vector<Type2>& Object3)
{
	//std::cout.precision(17);
	if(CGAL::squared_distance(Object1,Object2)<=threshold)
	{
		//std::cout<<"C"<<std::endl;
		Object3.push_back(Object2);
		//std::cout<<"2.1"<<std::endl;
	}
}
bool on_Line(Point_2 A, Point_2 B, Point_2 C)
{
	//std::cout.precision(17);
	long double dxc = CGAL::to_double(C.x() - A.x());
	long double dyc = CGAL::to_double(C.y() - A.y());

	long double dx1 = CGAL::to_double(B.x() - A.x());
	long double dy1 = CGAL::to_double(B.y() - A.y());

	double cross = CGAL::to_double((dxc * dy1) - (dyc * dx1));
	
	if(cross == 0)
	{
		return 1;
	}

	if(CGAL::abs(dx1) >= CGAL::abs(dy1))
	{
		if(dx1 > 0)
		{
			return (A.x() <= C.x() && C.x() <= B.x());
		}
		else
		{
			return (B.x() <= C.x() && C.x() <= A.x());
		}
	}
	else
	{
		if(dy1 > 0)
		{
			return (A.y() <= C.y() && C.y() <= B.y());
		}
		else
		{
			return (B.y() <= C.y() && C.y() <= A.y());
		}
	}

}

//function to get the inside points for a given threshold
std::vector<Point_2> insidePoints(const QuadTree& tree,const Segment_2& edge,const double& thresh)
{
	//std::cout.precision(17);
	std::vector<std::vector<Point_2> > tempPoints;
	tempPoints.reserve(5);
	std::queue<boost::shared_ptr<Node>> nodeQueue;
	//std::unordered_map<Point_2,int> mapping;
	//int count=0;
	
	std::vector<Point_2> th_Points;
	th_Points.reserve(10);
	boost::shared_ptr<Node> temp;
	//temp.reserve(4);
	
	//std::cout<<"insidePoint"<<std::endl;
	std::vector<Segment_2> tempSeg=bBox(edge,thresh);
	//std::cout<<"bBox"<<std::endl;
	
	if (tree.rootNode==nullptr){return th_Points;}
	else{
	
		nodeQueue.push(tree.rootNode);
		while(!nodeQueue.empty()){
			temp=nodeQueue.front();
			
			if(temp->child.size()==0)
			{
				tempPoints.push_back(temp->insidePoints);
			}

			for(int i=0;i<temp->child.size();++i)
			{	
				 Segment_2 Rect1 = Segment_2(temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1));
				 Segment_2 Rect2 = Segment_2(temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2));
				 Segment_2 Rect3 = Segment_2(temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3));
				 Segment_2 Rect4 = Segment_2(temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0));
			//	std::cout<<"1"<<std::endl;
				bool boxInside=true;
				if((CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(0))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(1))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(2))<=thresh)||(CGAL::squared_distance(tempSeg.at(2),temp->child.at(i)->rectangle.vertex(3))<=thresh))
				{					
					boxInside=true;
				}
				
				else
				{
					boxInside=false;
				}


				/*
			if ((CGAL::do_intersect(tempSeg, temp->child.at(i)->cuboid.bbox())) || (boxInside == true)) {
					nodeQueue.push(temp->child.at(i));
						}

				*/
				
				Segment_2 seg1 = tempSeg.at(0);
				Segment_2 seg2 = tempSeg.at(1);
				Segment_2 seg3 = tempSeg.at(2);

				if ((seg1.is_degenerate() || seg2.is_degenerate() || seg3.is_degenerate()) == 0)
				{

				//cout<<tempSeg.at(0).source().x()<<" "<<tempSeg.at(0).source().y()<<"      "<<tempSeg.at(0).end().x()<<" "<<tempSeg.at(0).end().y()<<endl;



					bool re =CGAL::do_intersect(tempSeg.at(0),temp->child.at(i)->rectangle);
					bool re1 =CGAL::do_intersect(tempSeg.at(1),temp->child.at(i)->rectangle);
					bool re2=CGAL::do_intersect(tempSeg.at(2),temp->child.at(i)->rectangle);


				if((CGAL::do_intersect(tempSeg.at(0),temp->child.at(i)->rectangle))||(CGAL::do_intersect(tempSeg.at(1),temp->child.at(i)->rectangle))||(CGAL::do_intersect(tempSeg.at(2),temp->child.at(i)->rectangle))||(boxInside==true))
				{					
					nodeQueue.push(temp->child.at(i));
					
				}
				}
				//else
				//{std::cout<<"In Inside Points (not in here)"<<std::endl;}
				/*else if(((CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1))) && (CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1)))) || (CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2)) && (CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2)))) || (CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3)) && (CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3)))) || (CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0)) || (CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0)))))

				//else if(CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1)) || CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2)) || CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3)) || CGAL::collinear(tempSeg.at(2).source(), temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0)) || CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1)) || CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2)) || CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3)) || CGAL::collinear(tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0)))
				{
					if((on_Line(tempSeg.at(2).source(), tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(0))) || (on_Line(tempSeg.at(2).source(), tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(1))) || (on_Line(tempSeg.at(2).source(), tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(2))) || (on_Line(tempSeg.at(2).source(), tempSeg.at(2).end(), temp->child.at(i)->rectangle.vertex(3))) 
						|| (on_Line(temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1), tempSeg.at(2).source())) || (on_Line(temp->child.at(i)->rectangle.vertex(0), temp->child.at(i)->rectangle.vertex(1), tempSeg.at(2).end())) || (on_Line(temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2), tempSeg.at(2).source())) || (on_Line(temp->child.at(i)->rectangle.vertex(1), temp->child.at(i)->rectangle.vertex(2), tempSeg.at(2).end())) 
						|| (on_Line(temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3), tempSeg.at(2).source())) || (on_Line(temp->child.at(i)->rectangle.vertex(2), temp->child.at(i)->rectangle.vertex(3), tempSeg.at(2).end())) || (on_Line(temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0), tempSeg.at(2).source())) || (on_Line(temp->child.at(i)->rectangle.vertex(3), temp->child.at(i)->rectangle.vertex(0), tempSeg.at(2).end())))
					{
						std::cout<<"Collinear Segment"<<std::endl;

						//tempPoints.push_back(temp->parent->insidePoints);
						//tempPoints.push_back(temp->

						Point_1 Sp_11 = Point_1(tempSeg.at(0).source().x(),tempSeg.at(0).source().y());
						Point_1 Sp_12 = Point_1(tempSeg.at(0).end().x(),tempSeg.at(0).end().y());

						Point_1 Sp_21 = Point_1(tempSeg.at(1).source().x(),tempSeg.at(1).source().y());
						Point_1 Sp_22 = Point_1(tempSeg.at(1).end().x(),tempSeg.at(1).end().y());
					
						Line_2 S1 = Line_2(Sp_11, Sp_12);
						Line_2 S2 = Line_2(Sp_21, Sp_22);					
					
						std::vector<Point_2> tempP;
						std::vector<Point>::iterator v_itr;
						for(v_itr = OriginalSample.begin();v_itr != OriginalSample.begin(); ++v_itr)
						{
							/*Segment_2 S1, S2;
							if(tempSeg.at(0).source().x() < tempSeg.at(1).source().x())
							{
								S1 = tempSeg.at(0);
								S2 = tempSeg.at(1);
							}
							else 
							{
								S1 = tempSeg.at(1);
								S2 = tempSeg.at(0);
							}
							
							if(S1.source().x() > S1.end().x())
							{
								//if((v_itr->x() >= S1.source().x() && v_itr->x() <= S2.source().x()) && (v_itr->y() >= S1.source().y() && v_itr->y() <= S2.source().y()))
								//Condition not correct, check it
								{
									tempP.push_back(*v_itr);
								}
							}
							Point_1 p = Point_1(v_itr->x(), v_itr->y());
						
							if(((CGAL::squared_distance(S1, p)) + (CGAL::squared_distance(S2, p))) <= CGAL::square(thresh))
							{
								tempP.push_back(*v_itr);
							}
						}
						return tempP;
					}
					
				}*/
				
				
			}
			nodeQueue.pop();
		}
	}
	for(int i=0;i<tempPoints.size();++i){
		//std::cout<<"2"<<std::endl;
		for(int j=0;j<tempPoints[i].size();++j){
			thresh_Points(edge,tempPoints[i].at(j),thresh,th_Points);
		}
	}
	std::vector<Point_2> th1_Points;
	th1_Points.reserve(10);
	//std::cout<<"done"<<std::endl;
	return th_Points;
}


//Funtion to Create Delaunay Triangulation from a file of points
void create_Delaunay(Delaunay& dt,std::vector<Point_2> &input)
{
	//std::cout.precision(17);
	//std::istream_iterator <Point_2> begin(input);
   	//std::istream_iterator <Point_2> end;
   	dt.insert(input.begin(),input.end());
   	
}

//To assign ids to the Delaunay Triangulation
void Assign_ids(Delaunay& dt)
{
	//std::cout.precision(17);
	int id = 0;
	Finite_vertices_iterator_2d vit;
	for (vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); vit++) {
		vit->id = id;
		id++;
	}
}

//Function to Output Delaunay Edges so as to view in Opengl
void outputDelaunayEdges(Delaunay dt,std::ofstream &output)
{
	//output.precision(17);
	/*Assign_ids(dt);
	
	output.precision(17);
	Finite_vertices_iterator_2d vit;
	for (vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); vit++) {
		output<<vit->point()<<std::endl;
	}*/

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
			//output<<"3 "<<first->id<<" "<<second->id<<" "<<third->id<<std::endl;
			output<<first->point()<<" ";output<<second->point()<<std::endl;
			output<<second->point()<<" ";output<<third->point()<<std::endl;
			output<<third->point()<<" ";output<<first->point()<<std::endl;
		}		
	}
  	output.close();
} 

//Function to Output Delaunay Edges so as to view in Opengl
void outputInsideDelaunayEdges(Delaunay dt,std::ofstream &output)
{
	//output.precision(17);
	//Finite_faces_iterator_2d face_iterator;
	fh f_handle = dt.incident_faces(dt.infinite_vertex());
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

	fit f1 =dt.finite_faces_begin();

	for(; f1 != dt.finite_faces_end(); f1++)
	{
		if(f1->face_is_inside==true)
		{			
			//++numInsideTri;
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

//Function to get nearest and next_nearest neighbors using NNCrust and store it in a vector in the form {point,nearest,next_nearest}
template <typename Type>
void NNCrust(Delaunay& dt,std::vector<Point_2>& sample,std::vector<vector <Point_2> >& Neighbors,std::vector<Segment_2>& seg,Type ray,const double& threshold )
{	
	//std::cout.precision(17);
	Finite_vertices_iterator_2d vit;
	std::vector< vector <Point_2> > TestNeigh;
	
	for(vit = dt.finite_vertices_begin();vit!=dt.finite_vertices_end();vit++)
	{
		bool notThresh_point=true;
		if(CGAL::squared_distance(ray,vit->point())<threshold)
		{
			notThresh_point=false;
		}
		else{notThresh_point=true;}
		
	if(notThresh_point==false)
	{
		int count = 0;
		//store pair of vertex 
		//for each vertex find the Delaunay neighbors.
		std::vector<vh> neighbors;
		std::vector<Point_2> TestP;
		TestP.push_back(vit->point());
		//get a face handle incident on vit
		fh f1 = vit->face();

		//find the index of vit in f1
		int index = f1->index(vit);
		int index1 = (index + 1)%3;
		int index2 = (index + 2)%3;
		
		Vertex_circulator vcirculator=dt.incident_vertices(vit),done(vcirculator);
		do
		{
			if(!dt.is_infinite(vcirculator)){neighbors.push_back(vcirculator);}
		} while(++vcirculator != done);

		for(int i=0;i<neighbors.size();i++)
		{
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
			double d= CGAL::to_double(squared_distance(TestNeigh[l].at(0),TestNeigh[l].at(j)));
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
			double dist = CGAL::to_double(squared_distance(p1,p3)); 
			
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
		TempNeighbors.push_back(TestNeigh[l].at(0));
		TempNeighbors.push_back(TestNeigh[l].at(nearest));
				
		TempNeighbors.push_back(TestNeigh[l].at(next_nearest));
		Neighbors.push_back(TempNeighbors);			
	}
	for(int p=0;p<Neighbors.size();p++)
	{
		Segment_2 segment1(Neighbors[p].at(0),Neighbors[p].at(1));
		Segment_2 segment2(Neighbors[p].at(0),Neighbors[p].at(2));
		seg.push_back(segment1);
		seg.push_back(segment2);
		
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
}

//Function to check number of intersections
template <typename Type1,typename Type2>
int check_Intersection(Type1 rays,std::vector<Type2> seg)
{
	//std::cout.precision(17);
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
						//points.push_back(CGAL::intersection(rays,seg.at(j));
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
	//std::cout.precision(17);
	//std::cout<<"Error get Farthest Point"<<std::endl;
	Point_2 Point;
	double d_max=0;
	for(int i=0;i<seg.size();i++)
	{
		//std::cout<<"Error get Farthest Point 2"<<std::endl;
		//std::vector<Point_2> tempVec;
		Point_2 far_point;
		if((CGAL::do_intersect(ray,seg.at(i)))&&(seg.at(i).source()!=seg.at(i).target()))
		{
			
			const Point_2* tempPoint;
			if(seg.at(i).is_vertical()){
					
				//std::cout<<" IsVertical segment is vertical"<<std::endl;
				double t =CGAL::to_double((seg.at(i).source().x()-ray.source().x())/ray.direction().dx());
				Point_2 pq(seg.at(i).source().x(),ray.source().y()+t*(ray.direction().dy()));
				far_point=pq;
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
			double dist=CGAL::to_double(CGAL::squared_distance(point,far_point));
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
	}
	return Point;

}
void check_duplicate(std::vector<Point_2> &input)
{
	//std::cout.precision(17);
	//std::vector<Point_2> th_pts;
	for(int j=0;j<input.size();++j)
	{
		for(int k=j;k<input.size();++k)
		{
			if(j==k){continue;}
			else
			{
				if(input.at(j)==input.at(k))
				{
					//ThreshPoints.erase(ThreshPoints.begin(),ThreshPoints.begin()+k);	--k;
					input.erase(input.begin()+k);	--k;
				}
				
			}
		}
	}
}
void mark_InsideFace(Delaunay& dt, const vh& point)
{
	Face_circulator fc = dt.incident_faces(point), done(fc);
	
	do
	{		
		if(fc->face_is_inside == true)
		{			
			//fc->face_is_inside = true;
			for(int i = 0; i < 3; i++)
			{
				if(fc->correct_segments[i]==false)
				{							
					if((fc->neighbor(i))->face_is_inside== false)
					{
						fc->face_is_inside = false;
					}
				}
	
			}
		}
	
	}while(++fc!=done);	
}

void deFace(Delaunay& dt, const vh& point)
{
	//std::cout.precision(17);
	//std::cout<<"2.1"<<std::endl;
	Face_circulator fc=dt.incident_faces(point),done(fc);
	
	do
	{
		//std::cout<<point->id<<" "<<point->point();
		for(int i = 0; i < 3; ++i)
		{
			//point->
			Edge e = Edge(fc,i);
			
			//if(e.first->correct_segments[e.second] == true)
			//{
			//if(dt.number_of_vertices() >=166)
				//std::cout<<"Unmarked Edge	"<<e.first->vertex((e.second+1)%3)->point()<<" "<<e.first->vertex((e.second+2)%3)->point()<<std::endl;
			//}			
			//if(!dt.is_infinite(e))
			{
			e.first->correct_segments[e.second]=false;
			fh opp_face = e.first->neighbor(e.second);
			int opp_index = opp_face->index(e.first);
			opp_face->correct_segments[opp_index] = false;
				}
			/*fc->correct_segments[i] = false;
			fh opp_face=fc->neighbor(i);

			int opp_index = opp_face->index(fc);
			opp_face->correct_segments[opp_index] = false;
				*/
		}
		
	}while(++fc!=done);
}
/*
void deFace_2(Delaunay& dt)
{
	all_fit a_fit;
	a_fit=dt.all_faces_begin();
	//int triangle_i = 1;
	for ( ; a_fit !=dt.all_faces_end();++a_fit) 
	{
	for(int i = 0; i < 3; i++)
		a_fit->correct_segments[i] = false;
		
		//triangle_i++;
	}
}
void deEdge(Edge& eit)
{
	eit.first->correct_segments[eit.second]=false;
				
	fh opp_face = eit.first->neighbor(eit.second);
	int opp_index = opp_face->index(eit.first);
	opp_face->correct_segments[opp_index] = false;
}*/

bool isSkinnyTriangle(fh faceItr)
{
	//std::cout.precision(17);
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
	circumradius = CGAL::to_double(CGAL::squared_radius(onePt, twoPt, threePt));
	//std::cout<<"circumradius is		"<<circumradius<<std::endl;						
	double dist1 = CGAL::to_double(CGAL::squared_distance(onePt, twoPt));
	double dist2 = CGAL::to_double(CGAL::squared_distance(twoPt, threePt));
	double dist3 = CGAL::to_double(CGAL::squared_distance(threePt, onePt)); 

	//c is the circumcenter of the concerned triangle
	Point c = CGAL::circumcenter(onePt, twoPt, threePt);
		//std::cout<<"Circumcenter is		"<<c<<std::endl;
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

bool is_circumcenter_inside(Point_2 p1, Point_2 p2, Point_2 p3, Point_2 q)
{
	//std::cout.precision(17);
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

void circumcenter_position(Delaunay& dt, fh faceItrs, bool &circumcenter_is_outside, Point_2& circum_center, Point_2& enchroch_Point, bool& flag)
{		
		//std::cout.precision(17);
		int not_to_check = -1;
		circumcenter_is_outside = true;		
		fh next_face;
		bool move_ahead = true;
//		Point_2 circum_center;

		vh first = (faceItrs->vertex(0));
		//unsigned int first_id = first->id; 

		vh second = (faceItrs->vertex(1));
		//unsigned int second_id = second->id;

		vh third = (faceItrs->vertex(2));
		//unsigned int third_id = third->id;	
			
		//Find centroid
		Point_2 centroid = CGAL::centroid(first->point(), second->point(), third->point());
		Point_2 c = CGAL::circumcenter(first->point(), second->point(), third->point());

		//std::cout<<"centroid	"<<centroid<<"	circumcenter	"<<c<<std::endl;
		Point_2 p1(centroid.x(), centroid.y());
		Point_2 p2(c.x(), c.y());
		// Line between centroid and circumcenter.
		Segment_2 l(p1, p2);	
		//std::cout<<"Centroid	"<<centroid<<std::endl;	
		//std::cout<<"Circumcenter	"<<c<<std::endl;	
		//int debug_counter = 0;
		Point_2 p_first, p_second, p_third;
		Segment_2 line_1, line_2, line_3;			

		while(move_ahead)
		{
			
			//std::cout<<" Inside loop "<<std::endl;					
			p_first = Point_2(first->point().x(), first->point().y());
			p_second = Point_2(second->point().x(), second->point().y());
			p_third = Point_2(third->point().x(), third->point().y());
			
			line_1 = Segment_2(p_first, p_second);  // Line between first and second point of triangle
			line_2 = Segment_2(p_second, p_third);	// Line between second and third point of triangle
			line_3 = Segment_2(p_third, p_first);	// Line between third and first point of triangle

			
			//std::cout<<"Line_1	"<<line_1.source()<<" "<<line_1.end()<<std::endl;
			//std::cout<<"Line_2	"<<line_2.source()<<" "<<line_2.end()<<std::endl;
			//std::cout<<"Line_3	"<<line_3.source()<<" "<<line_3.end()<<std::endl;

			// FIRST CHECK IF CIRCUMCENTER IS INSIDE THIS TRIANGLE
			if(is_circumcenter_inside(p_first, p_second, p_third, p2))
			{
				//std::cout<<" is_circumcenter_inside is true "<<std::endl;
				if(!faceItrs->face_is_inside)		//face is outside
				{
					circumcenter_is_outside = true;
				}
				else
				{
					circumcenter_is_outside = false;
				}
				move_ahead = false;
				break;
			}
			else 
			{
				if((!CGAL::do_intersect(l, line_1)) && (!CGAL::do_intersect(l, line_2)) && (!CGAL::do_intersect(l, line_3)))
				{
					std::cout<<"No Intersection in circumcenter position"<<std::endl;
					circumcenter_is_outside = false;					
					move_ahead = false;
					break;
				}			
				//std::cout<<"cirumcenter is not inside and not_to_check = "<<not_to_check<<std::endl;
				if(not_to_check==-1 || not_to_check != 2){
					//std::cout<<" 1. "<<std::endl;
					if(CGAL::do_intersect(l, line_1) == true)
					{
						//std::cout<<"  1"<<std::endl;
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
						//std::cout<<"  2"<<std::endl;
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
						//std::cout<<"  3"<<std::endl;
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
		//circumcenter_is_outside = !circumcenter_is_outside; 
		//if(circumcenter_is_outside == false)
		//{
			//flag = false;
			/*std::vector<fh> faces;
			std::vector<fh>::iterator face_it;

			for(int c=0; c < 3; ++c)
			{
				faces.push_back(faceItrs->neighbor(c));
			}

			for(face_it = faces.begin(); face_it != faces.end(); ++face_it)
			{
				fh face = *face_it;
				for(int k = 0; k < 3; ++k)
				{
					Edge e1 = Edge(face, k);
					if(e1.first->correct_segments[e1.second] == true)
					{
						Point_2 midPoint = CGAL::midpoint(e1.first->vertex((e1.second+1)%3)->point(), e1.first->vertex((e1.second+2)%3)->point());
						if(CGAL::squared_distance(midPoint, e1.first->vertex((e1.second+1)%3)->point()) >= CGAL::squared_distance(midPoint, c))
						{
							flag = true;
							enchroch_Point.push_back(midPoint);
							std::cout<<"Enchrochment"<<std::endl;
							//return circum_center;
						}					
					}					
				}
			}*/
		
			flag = false;
			for(Edge_iterator et = dt.finite_edges_begin(); et != dt.finite_edges_end(); ++et)
			{
				if(et->first->correct_segments[et->second] == true)
				{
					Point_2 midPoint = CGAL::midpoint(et->first->vertex((et->second+1)%3)->point(), et->first->vertex((et->second+2)%3)->point());
					if(CGAL::squared_distance(midPoint, et->first->vertex((et->second+1)%3)->point()) >= CGAL::squared_distance(midPoint, c))
						{
							flag = true;
							enchroch_Point = midPoint;
							//std::cout<<"Enchrochment"<<std::endl;
							break;
							//return circum_center;
						}	
				}
			}
			/*for(int k = 0; k < 3; ++k)
			{
				Edge e1 = Edge(faceItrs, k);
				if(e1.first->correct_segments[e1.second] == true)
				{
					Point_2 midPoint = CGAL::midpoint(e1.first->vertex((e1.second+1)%3)->point(), e1.first->vertex((e1.second+2)%3)->point());
					if(CGAL::squared_distance(midPoint, e1.first->vertex((e1.second+1)%3)->point()) >= CGAL::squared_distance(midPoint, c))
					{
						flag = true;
						enchroch_Point = midPoint;
						//std::cout<<"Enchrochment"<<std::endl;
						//return circum_center;
					}					
				}					
			}*/
			circum_center=c;
			//std::cout<<"Circumcenter is		"<<circumcenter_is_outside<<std::endl;
								
		//return faceItrs;
}

/*void get_edges(Delaunay& dt, vh& temp, std::vector<Edge>& edges)
{
	Edge_circulator ec = dt.incident_edges(temp), done(ec);
	do
	{
		edges.push_back(*ec);
	}while(++ec != done);
}*/
/*
void get_faces(Delaunay& dt, vh& temp, std::vector<Edge>& edges)
{
	Face_circulator fc = dt.incident_faces(temp), done(fc);
	do
	{// Edge *e = new Edge();
		//if(fc != dt.infinite_face())
		if(fc->vertex(1) != dt.infinite_vertex() && fc->vertex(2) != dt.infinite_vertex())
		for(int i = 0; i <3; ++i)	
		{
			//std::cout<<Edge(fc, i).first->vertex((Edge(fc, i).second+1)%3)->point()<<" "<<Edge(fc, i).first->vertex((Edge(fc, i).second+2)%3)->point()<<std::endl;
			edges.push_back(Edge(fc, i));
		}
	}while(++fc != done);
}
/*
bool get_dual(Delaunay& dt, Edge& e, const QuadTree& tree, Segment_2& temp)
{
	bool v;
	CGAL::Object o = dt.dual(e);			

			const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
			const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
			if (r)
        	{  //4
				//std::cout<<"Ray"<<std::endl;
				//outputRays<<*r<<std::endl;				
        		
				if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
					temp=convToSeg(tree.rootNode->rectangle,*r);
				}
				v = 1;
			}
			if (s) 
        	{	//4
				//std::cout<<"Seg"<<std::endl;
				//outputRays<<*s<<std::endl;
				v = 0;
				temp = *s;	
			}
			return v;			
}*/
/*
int Topology(Delaunay& dt, const QuadTree& tree, Edge& e, bool& rays_ok, bool& segs_ok, Segment_2& temp, std::vector<Segment_2>& Neighbor_Segments)
{
	
			int num_of_intersections=0;
			std::vector<Point_2> ThreshPoints, NewThreshPoints;
			std::vector<vector <Point_2> > Neighbors;
		
			CGAL::Object o = dt.dual(e);			

			const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
			const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
			if (r)
        	{  //4
				//std::cout<<"Ray"<<std::endl;
				//outputRays<<*r<<std::endl;				
        		
				if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
					temp=convToSeg(tree.rootNode->rectangle,*r);
				}
				
			}
			if (s) 
        	{	//4
				//std::cout<<"Seg"<<std::endl;
				//outputRays<<*s<<std::endl;
				
				temp = *s;	
			}
			
			ThreshPoints = insidePoints(tree, temp, 1500);
			check_duplicate(ThreshPoints);
			
			Delaunay dt_thresh;
			create_Delaunay(dt_thresh,ThreshPoints);		
			Assign_ids(dt_thresh);
								
			NewThreshPoints = insidePoints(tree, temp, 1200);
			check_duplicate(NewThreshPoints);
			
			//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
			if(NewThreshPoints.size() > 0)
			{						
				NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,temp,1200);
				num_of_intersections = check_Intersection(temp,Neighbor_Segments);
			}
			if(num_of_intersections>1)
			{
				if(r)
					rays_ok = false;
				if(s)
					segs_ok = false;
			}
			return num_of_intersections;
}*/
/*void Topology(Delaunay& dt, const QuadTree& tree, bool& rays_ok, bool& segs_ok, Segment_2& temp, std::vector<Point_2>& intersection_points, bool v)
{			
			int num_of_intersections=0;
			std::vector<Point_2> ThreshPoints, NewThreshPoints;
			std::vector<vector <Point_2> > Neighbors;
			std::vector<Segment_2> Neighbor_Segments;
			ThreshPoints = insidePoints(tree, temp, 1500);
			check_duplicate(ThreshPoints);
			
			Delaunay dt_thresh;
			create_Delaunay(dt_thresh,ThreshPoints);		
			Assign_ids(dt_thresh);
								
			NewThreshPoints = insidePoints(tree, temp, 1200);
			check_duplicate(NewThreshPoints);
			
			//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
			if(NewThreshPoints.size() > 0)
			{						
				NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,temp,1200);
				check_Intersection(temp,intersection_points);
			}
			if(intersection_points.size()>1)
			{
				if(v)
					rays_ok = false;
				else
					segs_ok = false;
			}
			//return intersection_points.size();
}*/

void markEdge(Delaunay& dt, const QuadTree& tree, const vh& point)
{	
	//std::ofstream HomoEdges;
	//HomoEdges.open("HomomorphicalEdges.txt",std::ofstream::out | std::ofstream::trunc);
	//HomoEdges.precision(17);
	//std::cout<<"	Marking Edge	"<<std::endl;
	Face_circulator fc=dt.incident_faces(point),done(fc);
	//if(!dt.is_infinite(fc))
	//{
		//std::cout<<"face is not infinite"<<std::endl;
	do
	{
		//std::cout<<point->id<<" "<<point->point();
		for(int i = 0; i < 3; ++i)
		{
			Edge e = Edge(fc,i);
			if(!dt.is_infinite(e))
			{
				CGAL::Object o = dt.dual(e);
					
				const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
				const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
			
				int num_of_intersections=0;
				std::vector<Point_2> ThreshPoints, NewThreshPoints;
				std::vector<vector <Point_2> > Neighbors;
				std::vector<Segment_2> Neighbor_Segments;
				Segment_2* temp = new Segment_2;
			
				if (r)
        		{  
					if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
						*temp=convToSeg(tree.rootNode->rectangle,*r);
						
					}
				}
				if (s) 
        		{	
					*temp = *s;	
				}
				/*if((dt.number_of_vertices() >= 235) && (dt.number_of_vertices() <= 237))
						{
							//std::cout<<"dual voronoi is		"<<temp->source()<<" "<<temp->end()<<std::endl;
						}*/
				ThreshPoints = insidePoints(tree, *temp, OT);
				check_duplicate(ThreshPoints);
			
				Delaunay dt_thresh;
				create_Delaunay(dt_thresh,ThreshPoints);		
				//Assign_ids(dt_thresh);
								
				NewThreshPoints = insidePoints(tree, *temp, IT);
				check_duplicate(NewThreshPoints);
			
				//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
				if(NewThreshPoints.size() > 1)
				{						
					NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*temp,IT);
					num_of_intersections=check_Intersection(*temp,Neighbor_Segments);
				}
			
				if(num_of_intersections==1)
				{		
					e.first->correct_segments[e.second]=true;
					//if(dt.number_of_vertices() >=166)
						//std::cout<<"Marked Edge	"<<e.first->vertex((e.second+1)%3)->point()<<" "<<e.first->vertex((e.second+2)%3)->point()<<std::endl;
									
					fh opp_face = e.first->neighbor(e.second);
					int opp_index = opp_face->index(e.first);
					opp_face->correct_segments[opp_index] = true;
				}	
			}
		}
	}while(++fc!=done);
	//}
	/*for(Edge_iterator eg = dt.finite_edges_begin(); eg != dt.finite_edges_end(); ++eg)
	{
		if((eg->first->correct_segments[eg->second] == true))
						 
		{
			HomoEdges<<eg->first->vertex((eg->second+1)%3)->point()<<" "<<eg->first->vertex((eg->second+2)%3)->point()<<std::endl;
		//std::cout<<eg->first->vertex((eg->second+1)%3)->point()<<" "<<eg->first->vertex((eg->second+2)%3)->point()<<std::endl;
		}
	}*/
}

int main()
{
	int check;
	std::clock_t t1,t2;
	t1=clock();
	//std::cout.precision(17);
	std::vector<Point_2> OriginalSample,RandomSample;
	
	//Inputs and Outputs from/to various files
	std::ifstream input("Simple2_New.txt");
	//std::ofstream outputRandomSample("RandomSample.txt");
	//outputRandomSample.precision(17);
	
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
	Iso_rectangle_2	boundingBox;
	MinMaxCoor(OriginalSample,boundingBox);
	QuadTree tree(boundingBox);
	
	tree.rootNode->insidePoints=OriginalSample;
	generate_QuadTree(tree,tree.rootNode);
	//printTree(tree);	
	//t3=clock();
//	if(con_input==1)
	{
	//Taking some random sample from the original sample
		for(int i=0;i<15;i++)
		{
			int n=std::rand()%(num_of_points-1);
			RandomSample.push_back(OriginalSample.at(n));
			//if(outputRandomSample.is_open()){outputRandomSample<<OriginalSample.at(n)<<std::endl;}
			//if(outputRandomSampleIndices.is_open()){outputRandomSampleIndices<<n<<std::endl;}
		}
	}

	/*std::ifstream inRandomSample;
	inRandomSample.open("SpecialCase.txt");
	std::string data1;
	while(getline(inRandomSample,data1))
	{
		Point_2 sample;
		std::istringstream stream(data1);
		while(stream>>sample)
		{
			RandomSample.push_back(sample);
			//if(outputRandomSample.is_open()){outputRandomSample<<sample<<std::endl;}			
		}
	}*/
	//Creating Delaunay Triangulation of the points in the inputRandomSample
	Delaunay dt_sample;
	create_Delaunay(dt_sample,RandomSample);		
	//Assign_ids(dt_sample);

	//int insideTri=0;
	std::vector<Point_2> ThreshPoints, NewThreshPoints;
	std::vector<vector <Point_2> > Neighbors;
	std::vector<Segment_2> Neighbor_Segments;
	std::multimap<Edge , Point_2> listing;
	bool iterate=true;
	//std::cout<<"Num of points	"<<dt_sample.number_of_vertices()<<std::endl;
	//std::cin>>check;

	while(iterate)
	{  //1
		//Assign_ids(dt_sample);
		//std::cin>>check;
				
		//std::ofstream HomoEdges;
		//HomoEdges.open("HomomorphicalEdges.txt",std::ofstream::out | std::ofstream::trunc);
		
		//std::ofstream outputRays;
		//outputRays.open("OutputRays.txt",std::ofstream::out | std::ofstream::trunc);
		
		//std::cout<<"Input 1 to continue iteration"<<std::endl;
		//int console_input;
		//std::cin>>console_input;

		//Ofstream for ouputPoints i.e the Threshold Points
		//std::ofstream outputPoints;
		
		std::cout<<".....................................Topology............................."<<std::endl;
					
		//std::ofstream outputDelaunay;
		//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
		//outputDelaunayEdges(dt_sample,outputDelaunay);
		
		listing.clear();
		//std::vector<vh> points;
		//std::vector<vh>::iterator v_it;
		//Vertex_iterator v_it;
		Edge_iterator eit =dt_sample.finite_edges_begin();	
		for ( ; eit !=dt_sample.finite_edges_end();++eit) 
		{//2
			if(eit->first->correct_segments[eit->second] == false)
			{
				iterate = false;
				//std::cout<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+1)%3)->point()<<std::endl;
				CGAL::Object o = dt_sample.dual(eit);
					
				const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
				const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
			
				int num_of_intersections=0;
				ThreshPoints.clear();
				NewThreshPoints.clear();
				Neighbors.clear();
				Neighbor_Segments.clear();
				Segment_2* temp = new Segment_2;
			
				if (r)
        		{  //4
					//std::cout<<"Ray"<<std::endl;
					//outputRays<<*r<<std::endl;				
        		
					if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
						*temp=convToSeg(tree.rootNode->rectangle,*r);
					}
				}
				if (s) 
        		{	//4
					//std::cout<<"Seg"<<std::endl;
					//outputRays<<*s<<std::endl;
				
					*temp = *s;	
				}
			
				//For flower_300 pts are 1500 n 1200
				ThreshPoints = insidePoints(tree, *temp, OT);			
				check_duplicate(ThreshPoints);
			
				//std::cout<<"1"<<std::endl;
				Delaunay dt_thresh;
				create_Delaunay(dt_thresh,ThreshPoints);		
				//Assign_ids(dt_thresh);
								
				NewThreshPoints = insidePoints(tree, *temp, IT);
				check_duplicate(NewThreshPoints);				
				//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
				
				if(NewThreshPoints.size() > 1)
				{						
					NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*temp,IT);
					num_of_intersections=check_Intersection(*temp,Neighbor_Segments);
					//std::cout<<"num of Intersection	"<<num_of_intersections<<std::endl;
				}			
			
				if(num_of_intersections>1)
				{  	
					Point_2 Far_Point=get_Farthest_Point(*temp,Neighbor_Segments,eit->first->vertex((eit->second+1)%3)->point());
					//std::cout<<"4"<<std::endl;
					listing.insert(std::pair<Edge , Point_2>(*eit, Far_Point));
				}  //5

				else if(num_of_intersections==1) 
				{
					//HomoEdges<<eit->first->vertex((eit->second+1)%3)->point()<<" "<<eit->first->vertex((eit->second+2)%3)->point()<<std::endl;
					//if(eit->first->correct_segments[eit->second] == false)
					//{
						eit->first->correct_segments[eit->second]=true;
						fh opp_face = eit->first->neighbor(eit->second);
						int opp_index = opp_face->index(eit->first);
						opp_face->correct_segments[opp_index] = true;
					//}
				}				
				delete temp;				
			}
		}//2
		
		//std::cin>>stop;
		if(!listing.empty())
		{
			iterate = true;		
			for (std::multimap<Edge , Point_2>::iterator m_it=listing.begin(); m_it!=listing.end(); ++m_it)
			{
				vh vh1, vh2;
				vh1 = ((*m_it).first).first->vertex((((*m_it).first).second+1)%3);
				vh2 = ((*m_it).first).first->vertex((((*m_it).first).second+2)%3);
				if((dt_sample.is_edge(vh1, vh2)) && (((*m_it).first).first->correct_segments[((*m_it).first).second] == false))
				//if(dt_sample.is_edge(vh1, vh2))
				{					
					vh tempVhandle=dt_sample.insert((*m_it).second);
					//std::cout<<"Point Inserted	"<<(*m_it).second<<std::endl;
					//if(outputRandomSample.is_open()){outputRandomSample<<(*m_it).second<<std::endl;}
					deFace(dt_sample, tempVhandle);	
					markEdge(dt_sample, tree, tempVhandle);					
				}
			}			
		}

		//nop2 = dt_sample.number_of_vertices();
		//std::cout<<"nop2	"<<dt_sample.number_of_vertices()<<std::endl;
		//std::cin>>stop;
		//if(nop1 != nop2)
			//iterate=true;		
		std::cout<<"After Topology Phase	"<<dt_sample.number_of_vertices()<<std::endl;
		
		//std::ofstream outputDelaunay;
		//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
		//outputDelaunayEdges(dt_sample,outputDelaunay);

		/*//if(dt_sample.number_of_vertices() >= 900)
		{
			std::ofstream outputDelaunayInside;
			outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
			//if(dt_sample.number_of_vertices() >= 178)
			//std::cin>>check;
		}//		
		*/
		
		if(iterate == false)
		{//3
			//std::cin>>check;
			std::cout<<"......................Manifold...................."<<std::endl;			
						
			//std::ofstream outputDelaunay;
			//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			//outputDelaunayEdges(dt_sample,outputDelaunay);
			
			//{
				listing.clear();
				//iterate = false;
				//flag = false;
				Point_2 Farthest_Point;	
				eit=dt_sample.finite_edges_begin();
				for ( ; eit !=dt_sample.finite_edges_end();eit++) 
				{  //4		
					//flag = false;
					//if(!dt_sample.is_infinite(eit->first->vertex(eit->second)))  continue;
					//std::cout<<"1"<<std::endl;
					if(eit->first->correct_segments[eit->second]==true)
					{  //5
						//std::cout<<"2"<<std::endl;
					
						for(int i = 1; i < 3; i++)
						{	//6
							//std::cout<<"i ="<<i<<std::endl;
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
							//std::cout<<"count is	"<<count<<std::endl;
							if((count != 2))
							{  //7
								
								//flag = true;
								Farthest_Point =vh_center->point();
								Edge_circulator circulator1=dt_sample.incident_edges(vh_center),done(circulator1);
								//if(circulator1.is_empty()) continue;
								do
								{  //8
									//std::cout<<"3"<<std::endl;
									if(circulator1->first->correct_segments[circulator1->second]==true)
									{  //9									
										CGAL::Object o = dt_sample.dual(circulator1);
										const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
										const Ray_2* r=CGAL::object_cast<Ray_2>(&o);
    	  								Point_2 Far_Point;
										ThreshPoints.clear();
										NewThreshPoints.clear();
										Neighbors.clear();
										Neighbor_Segments.clear();
										Segment_2 *temp = new Segment_2;
        			
										if(r)
        								{  //10
											//std::cout<<"Ray"<<std::endl;
											//outputRays<<*r<<std::endl;
        									if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
												*temp=convToSeg(tree.rootNode->rectangle,*r);
											}
										}
										if(s)
        								{  //10
											//std::cout<<"Seg"<<std::endl;
											//outputRays<<*s<<std::endl;
											*temp = *s;
										}
										ThreshPoints = insidePoints(tree, *temp, OT);
										check_duplicate(ThreshPoints);
					
										Delaunay dt_thresh;
										create_Delaunay(dt_thresh,ThreshPoints);		
									
										NewThreshPoints = insidePoints(tree, *temp, IT);
										check_duplicate(NewThreshPoints);
										
										//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
									
										if( NewThreshPoints.size() > 1)
										NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*temp,IT);
										Far_Point=get_Farthest_Point(*temp,Neighbor_Segments,vh_center->point());
					
										if((CGAL::squared_distance(vh_center->point(),Far_Point)) > (CGAL::squared_distance(vh_center->point(),Farthest_Point)))
        								{
        									Farthest_Point=Far_Point;
											
        								}
										listing.insert(std::pair<Edge , Point_2>(*eit, Farthest_Point));
										delete temp;
									}
								}while(++circulator1 != done);//8
								
								/*vh tempVhandle=dt_sample.insert(Farthest_Point);	
								std::cout<<"Point Inserted	"<<Farthest_Point<<std::endl;
								iterate = true;
								deFace(dt_sample, tempVhandle);			
								if(outputRandomSample.is_open()){outputRandomSample<<Farthest_Point<<std::endl;}
								markEdge(dt_sample, tree, tempVhandle);*/

								break;
								//std::cout<<"	Marking done	"<<std::endl;	
							}//7
						}//6						
					}//5
					//if(flag == true)
							//break;
				}//4
				if(!listing.empty())
				{
					iterate = true;
					//flag = true;
					for (std::multimap<Edge , Point_2>::iterator m_it=listing.begin(); m_it!=listing.end(); ++m_it)
					{
						vh vh1, vh2;
						vh1 = ((*m_it).first).first->vertex((((*m_it).first).second+1)%3);
						vh2 = ((*m_it).first).first->vertex((((*m_it).first).second+2)%3);
						if(dt_sample.is_edge(vh1, vh2))
						{
							//if(dt_sample.number_of_vertices() >= 236)
							//if(dt_sample.number_of_vertices() >= 200)
							//std::cout<<"Point Inserted	"<<(*m_it).second<<std::endl;
							
							vh tempVhandle=dt_sample.insert((*m_it).second);	
							//if(outputRandomSample.is_open()){outputRandomSample<<(*m_it).second<<std::endl;}
							//std::cout<<"Point Inserted	"<<(*m_it).second<<std::endl;
							deFace(dt_sample, tempVhandle);									
							markEdge(dt_sample, tree, tempVhandle);
							
						}
					}					
								
				}
				//std::cout<<"In Manifold	"<<dt_sample.number_of_vertices()<<std::endl;
			//}		
			std::cout<<"After Manifold Phase	"<<dt_sample.number_of_vertices()<<std::endl;	
			//std::ofstream outputDelaunay;
			//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			//outputDelaunayEdges(dt_sample,outputDelaunay);
			/*if(dt_sample.number_of_vertices() >= 250)
			{
				std::ofstream outputDelaunayInside;
				outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
				outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
				//std::cin>>check;
			}*/
			
			
		}//3				
		
		if(iterate == false)
		{//4
			std::cout<<"......................Geometric...................."<<std::endl;	
			//std::cin>>check;
				listing.clear();
				Point_2 Far_Point;
				//flag = false;				
				
				eit=dt_sample.finite_edges_begin();
				for ( ; eit !=dt_sample.finite_edges_end();++eit) 
				{  //5
					//std::cout<<"1"<<std::endl;
					
					if(eit->first->correct_segments[eit->second]==true)
					{  //6
						
						//std::cout<<"2"<<std::endl;
						for(int i = 1; i < 3; ++i)
						{	//7						
							int count=0;
							std::vector<Edge_circulator> edge_cir;
							vh vh_center = eit->first->vertex((eit->second+i)%3);		
							vh vh_adj,vh_adj1;		

							Edge_circulator circulator=dt_sample.incident_edges(vh_center),done(circulator);				
							//std::cout<<"vh_center	"<<vh_center->point()<<std::endl;
							do
							{ //8
								if(circulator->first->correct_segments[circulator->second]==true)
								{  //9
									//if(count<1){circulator_edge=circulator;}
									//if(count==1){circulator_edge1=circulator;}
									
									if(vh_center==circulator->first->vertex((circulator->second+1)%3))
									{
										if(count<1)
										{
											vh_adj=circulator->first->vertex((circulator->second+2)%3);
											//std::cout<<"vh_adj	"<<vh_adj->point()<<std::endl;
										}
										if(count==1)
										{
											vh_adj1=circulator->first->vertex((circulator->second+2)%3);
											//std::cout<<"vh_adj1	"<<vh_adj1->point()<<std::endl;
										}
									}
									if(vh_center==circulator->first->vertex((circulator->second+2)%3))
									{
										if(count<1)
										{
											vh_adj=circulator->first->vertex((circulator->second+1)%3);
											//std::cout<<"vh_adj	"<<vh_adj->point()<<std::endl;
										}
										if(count==1)
										{
											vh_adj1=circulator->first->vertex((circulator->second+1)%3);
											//std::cout<<"vh_adj1	"<<vh_adj1->point()<<std::endl;
										}
									}
									edge_cir.push_back(circulator);
									++count;
								 }  //9
							}while(++circulator!=done); //8
							
							
							if((CGAL::squared_distance(vh_center->point(), vh_adj->point()) < 3) && (CGAL::squared_distance(vh_center->point(), vh_adj1->point()) < 3))
							
							{  
								edge_cir.clear();
								//std::cout<<"1.1"<<std::endl;
								break;
							} 							
							if(vh_center==nullptr){std::cout<<"it is null"<<std::endl;}
							if(vh_adj==nullptr){std::cout<<"it is null 1"<<std::endl;}
							if(vh_adj1==nullptr){std::cout<<"it is null 2"<<std::endl;}
							
							
							//std::cout<<"vh center	"<<vh_center->point()<<"	"<<vh_adj->point()<<"	"<<vh_adj1->point()<<std::endl;

							double vec1= CGAL::to_double((vh_center->point().x()-vh_adj->point().x())*(vh_center->point().x()-vh_adj1->point().x()));
							double vec2=CGAL::to_double((vh_center->point().y()-vh_adj->point().y())*(vh_center->point().y()-vh_adj1->point().y()));
							/*
							double vec11 = CGAL::to_double(vh_center->point().x()-vh_adj->point().x());
							double vec12 = CGAL::to_double(vh_center->point().y()-vh_adj->point().y());
							double vec21 = CGAL::to_double(vh_center->point().x()-vh_adj1->point().x());
							double vec22 = CGAL::to_double(vh_center->point().y()-vh_adj1->point().y());
							*/
							//std::cout<<"vec1	"<<vec1<<"	vec2	"<<vec2<<std::endl;
							double denom = CGAL::to_double(((CGAL::sqrt((CGAL::to_double(vh_center->point().x()-vh_adj->point().x())*CGAL::to_double(vh_center->point().x()-vh_adj->point().x()))+CGAL::to_double(vh_center->point().y()-vh_adj->point().y())*CGAL::to_double(vh_center->point().y()-vh_adj->point().y())))) * ((CGAL::sqrt((CGAL::to_double(vh_center->point().x()-vh_adj1->point().x())*CGAL::to_double(vh_center->point().x()-vh_adj1->point().x()))+CGAL::to_double(vh_center->point().y()-vh_adj1->point().y())*CGAL::to_double(vh_center->point().y()-vh_adj1->point().y())))));
							//double denom = CGAL::to_double(CGAL::sqrt((vec11 * vec11) + (vec12 * vec12)) + CGAL::sqrt((vec21 * vec21) + (vec22 * vec22)));
							double ang;
							//std::cout<<"denom	"<<denom;
							if(denom!=0)
							{
								ang=std::acos((vec1+vec2)/denom);
							}
							
							if(ang<(160*3.14)/180)
							{  //8 
								//flag = true;
								//std::cout<<"1"<<std::endl;
								double dist=CGAL::to_double(CGAL::squared_distance(edge_cir.at(0)->first->vertex((edge_cir.at(0)->second+1)%3)->point(),edge_cir.at(0)->first->vertex((edge_cir.at(0)->second+2)%3)->point()));
								double dist1=CGAL::to_double(CGAL::squared_distance(edge_cir.at(1)->first->vertex((edge_cir.at(1)->second+1)%3)->point(),edge_cir.at(1)->first->vertex((edge_cir.at(1)->second+2)%3)->point()));
								Edge_circulator circulator1;

								if(dist<=dist1){circulator1=edge_cir.at(1);}
								if(dist>dist1){circulator1=edge_cir.at(0);}
							
								if(circulator1==NULL){std::cout<<"It is NULL "<<std::endl;}
								CGAL::Object o = dt_sample.dual(circulator1);
							
								const Segment_2* s=CGAL::object_cast<Segment_2>(&o);
								const Ray_2* r=CGAL::object_cast<Ray_2>(&o);    		  				

								ThreshPoints.clear();
								NewThreshPoints.clear();
								Neighbors.clear();
								Neighbor_Segments.clear();
								Segment_2 *temp = new Segment_2;
        			
								if(r)
        						{  //10
									//std::cout<<"Ray"<<std::endl;
									//outputRays<<*r<<std::endl;
        							if(tree.rootNode->rectangle.has_on_bounded_side((*r).source())){
										*temp=convToSeg(tree.rootNode->rectangle,*r);
									}
								}
								if(s)
        						{  //10
									//std::cout<<"Seg"<<std::endl;
									//outputRays<<*s<<std::endl;
									*temp = *s;
								}
								ThreshPoints = insidePoints(tree, *temp, OT);
								check_duplicate(ThreshPoints);
					
								Delaunay dt_thresh;
								create_Delaunay(dt_thresh,ThreshPoints);		
								//Assign_ids(dt_thresh);											
					
								NewThreshPoints = insidePoints(tree, *temp, IT);
								check_duplicate(NewThreshPoints);
					
								//if(dt_sample.number_of_vertices() >= 570)
								//std::cout<<"Thresh Points "<<ThreshPoints.size()<<"		NewThreshPoints		"<<NewThreshPoints.size()<<std::endl;
								
								if( NewThreshPoints.size() > 1)
								NNCrust(dt_thresh,NewThreshPoints,Neighbors,Neighbor_Segments,*temp,IT);
								Far_Point=get_Farthest_Point(*temp,Neighbor_Segments,vh_center->point());
								//if((CGAL::squared_distance(vh_center->point(), Far_Point) >= 1) && (CGAL::squared_distance(Far_Point, vh_adj1->point()) >= 1) && (CGAL::squared_distance(Far_Point, vh_adj->point()) >= 1))
								//{
									listing.insert(std::pair<Edge , Point_2>(*eit, Far_Point));
					    			delete temp;								
									break;
								//}
								
								
							}//8
							edge_cir.clear();
						}//7
					}//6
					//if(flag == true)
						//break;
				}//5

				if(!listing.empty())
				{
					iterate = true;
					//flag = true;
					
					for (std::multimap<Edge , Point_2>::iterator m_it=listing.begin(); m_it!=listing.end(); ++m_it)
					{						
						vh vh1, vh2;
						vh1 = ((*m_it).first).first->vertex((((*m_it).first).second+1)%3);
						vh2 = ((*m_it).first).first->vertex((((*m_it).first).second+2)%3);
						if(dt_sample.is_edge(vh1, vh2))
						{		
							//if(dt_sample.number_of_vertices() >= 350)
							{
								//std::cin>>check;
							}
							vh tempVhandle=dt_sample.insert((*m_it).second);
							//if(outputRandomSample.is_open()){outputRandomSample<<(*m_it).second<<std::endl;}
							//std::cout<<"Point Inserted	"<<(*m_it).second<<std::endl;
							//std::cout<<"Num of Points	"<<dt_sample.number_of_vertices()<<std::endl;
							deFace(dt_sample, tempVhandle);							
							markEdge(dt_sample, tree, tempVhandle);
							//if(dt_sample.number_of_vertices() >= 179)
							//std::cin>>check;
						}
						
						//std::cout<<"number of points		"<<dt_sample.number_of_vertices()<<std::endl;
						//if((dt_sample.number_of_vertices() == 176) || (dt_sample.number_of_vertices() == 186) || (dt_sample.number_of_vertices() == 196) || (dt_sample.number_of_vertices() == 206) || (dt_sample.number_of_vertices() == 216) || (dt_sample.number_of_vertices() == 226) || (dt_sample.number_of_vertices() >= 232))
							//std::cin>>check;
					}
					//std::cout<<"In Geometric 	"<<dt_sample.number_of_vertices()<<std::endl;
					
					//std::ofstream outputDelaunay;
					//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
					//outputDelaunayEdges(dt_sample,outputDelaunay);									
				}				
			//}
			std::cout<<"After Geometric Phase	"<<dt_sample.number_of_vertices()<<std::endl;			
			//std::ofstream outputDelaunay;
			//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			//outputDelaunayEdges(dt_sample,outputDelaunay);
			/*std::ofstream outputDelaunayInside;
			outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
			*///std::cin>>check;
			//if(dt_sample.number_of_vertices() >= 261)
			//{
				//std::cin>>check;
			//}
		}//4		
		
		
		if(iterate==false)
		{			
			std::cout<<"..............................................Mesh Refinement.................................... "<<std::endl;
			//std::cin>>check;
			//std::vector<Point_2> CircumcenterPoint;
			//CircumcenterPoint.clear();
			//std::vector<Point_2>::iterator Cc_itr;
			Point_2 P;			
			//neha = false;			
			
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
				/*int count =0, count1 = 0;
				for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); ++face_iterator)
				{						
					if(face_iterator->face_is_inside == true)
					{		
						++count;
						bool skinny_triangle = false;
						skinny_triangle = isSkinnyTriangle(face_iterator);
						if(skinny_triangle)
						{
							++count1;
						}
					}
				}
				std::cout<<"Total Inside triangles are	"<<count<<std::endl;
				std::cout<<"Number of skinny triangles are	"<<count1<<std::endl;
				*/
			//std::cin>>check;
			bool flag1 = true;
			while(flag1)
			{
				flag1 = false;	
				bool flag = false;
				
				

				//std::vector<Point_2> enchroch_Point;
				//std::vector<Point_2>::iterator e_Pt;

				//std::cout<<"2"<<std::endl;
				
				for(face_iterator = dt_sample.finite_faces_begin(); face_iterator != dt_sample.finite_faces_end(); ++face_iterator)
				{						
					if(face_iterator->face_is_inside == true)
					{	
						if(CGAL::area(face_iterator->vertex(0)->point(), face_iterator->vertex(1)->point(), face_iterator->vertex(2)->point()) >= 0.0000001)
						{					
							bool skinny_triangle = false;
							bool Circumcenter_is_outside = false;
						
							Point_2 circum_center, enchroch_Point;
							skinny_triangle = isSkinnyTriangle(face_iterator);
							if(skinny_triangle)
							{							
								//std::cout<<face_iterator->vertex(0)->point()<<" "<<face_iterator->vertex(1)->point()<<" "<<face_iterator->vertex(2)->point()<<std::endl;
								circumcenter_position(dt_sample, face_iterator, Circumcenter_is_outside, circum_center, enchroch_Point, flag);
								if (flag == true)
								{
									//std::cout<<"Enchrochment"<<std::endl;
									P = enchroch_Point;
									flag1 = true;								
									break;
								}
								else if(Circumcenter_is_outside == false)
								{
									P = circum_center;
									//std::cout<<"Circumcenter"<<std::endl;	
									flag1 = true;								
									break;
								}							
							}
						}
					}
				}
				if(flag1 == true)
				//if(!CircumcenterPoint.empty())
				{
					//iterate=true;
					//flag1 = true;	
					//for (Cc_itr = CircumcenterPoint.begin(); Cc_itr != CircumcenterPoint.end(); ++Cc_itr)
					//{
					//if(flag == false)
					//{
						vh tempVhandle=dt_sample.insert(P);		
						//std::cout<<"Point Inserted	"<<P<<std::endl;
						deFace(dt_sample,tempVhandle);
						markEdge(dt_sample, tree, tempVhandle);
						
						mark_InsideFace(dt_sample, tempVhandle);
						//if(outputRandomSample.is_open()){outputRandomSample<<P<<std::endl;}
					//}
					/*if(flag == true)
					{
						vh tempVhandle=dt_sample.insert(P);		
						//std::cout<<"Point Inserted	"<<P<<std::endl;
						deFace(dt_sample,tempVhandle);
						markEdge(dt_sample, tree, tempVhandle);
						
						mark_InsideFace(dt_sample, tempVhandle);
						//if(outputRandomSample.is_open()){outputRandomSample<<P<<std::endl;}
					}*/		
					//std::cout<<"num of Points	"<<dt_sample.number_of_vertices()<<std::endl;
					//std::ofstream outputDelaunayInside;
					//outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
					//outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
					//std::ofstream outputDelaunay;
					//outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
					//outputDelaunayEdges(dt_sample,outputDelaunay);
					
					//if(dt_sample.number_of_vertices() >= 800)
					//{						
						//std::cin>>check;
					//}
					
				}
				//std::cout<<"4"<<std::endl;
			}			
			/*std::ofstream outputDelaunay;
			outputDelaunay.open("DelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			outputDelaunayEdges(dt_sample,outputDelaunay);			
			
			std::ofstream outputDelaunayInside;
			outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
			outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
			*/			
			//if(dt_sample.number_of_vertices() == 1132)
			//std::cin>>check;
			
			//std::cout<<"After Mesh Refinement Phase	"<<dt_sample.number_of_vertices()<<std::endl;
			
		}		
	}  //1
	//int c=0;
	std::ofstream outputDelaunayInside;
	outputDelaunayInside.open("InsideDelaunayEdges.txt",std::ofstream::out | std::ofstream::trunc);
	outputInsideDelaunayEdges(dt_sample,outputDelaunayInside);
	
	std::cout<<"Total Points = "<< dt_sample.number_of_vertices()<<std::endl;
	std::cout<<"Total Faces = "<< dt_sample.number_of_faces()<<std::endl;
	t2=clock();
	float timeSec=((float)(t2)-(float)(t1))/CLOCKS_PER_SEC;
	std::cout<<"Time Taken for Execution: "<<timeSec<<std::endl;
	
	int k;
	std::cin>>k;
	return 0;
}
