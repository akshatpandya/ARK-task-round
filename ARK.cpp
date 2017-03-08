#include <stdio.h>
#include <iostream>
#include <math.h>
#include <utility>
#include <stack>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef pair<int, int> Pair;
typedef pair<double, pair<int, int> > Pair2;
 
//structure for storing the values in my graph
struct node
{
    int from_x, from_y;
    double f, g, h;
};
 
 //to check if the corresponding point lies in the matrix
bool isValid(int row, int col, int m, int n)
{
    return (row >= 0) && (row < m) && (col >= 0) && (col < n);
}

//calculating the shortestdistance by A*

void astar()
{
	int i, j, m, n, check_value,Xstart=0, Ystart=0, Xend=0, Yend=0;
	//reading the number of rows and columns 
	cin>>m;
	cin>>n;	
	
    //grid that stores the position of obstacles and free spaces
    int **grid = (int**)malloc(m * sizeof(int*));
    for(i=0;i<m;i++)
		grid[i]= (int*)malloc(n * sizeof(int));
		
	for(i=0; i<m; i++)
    {
		for(j=0; j<n; j++)
		{
			cin>>check_value;
			if(check_value==1)
				grid[i][j] = 1;
			else if(check_value==0)
				grid[i][j] = 0;
			else if(check_value==5)
			{
				Xstart = i;
				Ystart = j;
			}
			else if(check_value==10)
			{
				Xend = i;
				Yend = j;
			}	
		}
	}
	

	Pair src = make_pair(Xstart, Ystart);
    Pair dest = make_pair(Xend, Yend);
	

    bool **closedList = (bool**)malloc(m * sizeof(bool*));
	for(i=0;i<m;i++)
		closedList[i]= (bool*)malloc(n * sizeof(bool));
   
    for(i=0; i<m; i++)
    {
		for(j=0; j<n; j++)
		{
			closedList[i][j] = false;
		}
	}
    
    
    
	node **graph = (node**)malloc(m * sizeof(node*));
	for(i=0;i<m;i++)
		graph[i]= (node*)malloc(n * sizeof(node));
    
//initialize value of each point in graph with these values 
    for (i=0; i<m; i++)
    {
        for (j=0; j<n; j++)
        {
            graph[i][j].f = FLT_MAX;
            graph[i][j].g = FLT_MAX;
            graph[i][j].h = FLT_MAX;
            graph[i][j].from_x = -1;
            graph[i][j].from_y = -1;
        }
    }

    i = src.first, j = src.second;
    graph[i][j].f = 0.0;
    graph[i][j].g = 0.0;
    graph[i][j].h = 0.0;
    graph[i][j].from_x = i;
    graph[i][j].from_y = j;

    set<Pair2> openList;
 //insert the starting point in the open list
    openList.insert(make_pair (0.0, make_pair (i, j)));
 

    bool foundDest = false;
 
    while (!openList.empty())
    {
        Pair2 p = *openList.begin();
 
        openList.erase(openList.begin());
	//mark the top most point in the open list as visited and pop it out
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
   
        double gNew, hNew, fNew;
 
		int k,l;
		//check its neighbours and update their distances and push them in the open list, they will automatically be sorted in ascending order according to their distances 'f'
		for(k=-1;k<=1;k++)
		{
			for(l=-1;l<=1;l++)
			{
				 if (isValid(i+k, j+l, m, n) == true && !(k==0 && l==0) && ((k*l==1 || k*l==-1)?0:1))
				 {
					 
					if ((i+k) == dest.first && (j+l) == dest.second)		//if we reach the destination, we start backtracing the path from the destination to the source and mark it 4 in the matrix
					{														//since we had updated the parent node for each node we can backtrace the path
						graph[i+k][j+l].from_x = i;
						graph[i+k][j+l].from_y = j;
						int row = dest.first;
						int col = dest.second;
						while (!(graph[row][col].from_x == row && graph[row][col].from_y == col ))
						{
							grid[row][col] = 4;
							int temp_row = graph[row][col].from_x;
							int temp_col = graph[row][col].from_y;
							row = temp_row;
							col = temp_col;
						}
						grid[row][col] = 4;
						foundDest = true;
						
						   for (i=0; i<m; i++)
							{
								for (j=0; j<n; j++)
								{
									cout<<grid[i][j]<<" ";
								}
								cout<<endl;
							}
						
						return;
					}

					else if (closedList[i+k][j+l] == false && grid[i+k][j+l] == 0)
					{
						gNew = graph[i][j].g + ((k==0||l==0)?1:1.414);
						hNew = (double)sqrt (((i+k)-dest.first)*((i+k)-dest.first) + ((j+l)-dest.second)*((j+l)-dest.second));
						fNew = gNew + hNew;
		 
						if (graph[i+k][j+l].f == FLT_MAX || graph[i+k][j+l].f > fNew)
						{
							openList.insert( make_pair(fNew,make_pair(i+k, j+l)));
							graph[i+k][j+l].f = fNew;
							graph[i+k][j+l].g = gNew;
							graph[i+k][j+l].h = hNew;
							graph[i+k][j+l].from_x = i;
							graph[i+k][j+l].from_y = j;
						}
					}
				}
			}
		}   
    }
    return;
}
 //we redirect the standard input and output to the files input2.txt and output2.txt
int main(int argc, char *argv[])
{
	int i;
		astar();
    return(0);
}
