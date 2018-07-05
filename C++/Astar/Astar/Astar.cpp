// astar_test.cpp : �������̨Ӧ�ó������ڵ㡣  
//  

#include "stdafx.h"  
#include <stdlib.h>  
#include <iostream>  
#include <vector>  
#include <algorithm>  
using namespace  std;




class CPoint
{
public:
	CPoint(int x, int y) :X(x), Y(y), m_pParentPoint(NULL), G(0), F(0), H(0)
	{

	}

	~CPoint()
	{
	}

	void calcF(){
		this->F = this->H + this->G;
	}
public:
	int X;
	int Y;
	int G;
	int H;
	int F;
	CPoint* m_pParentPoint;
};

bool CompF(const CPoint* pl, const CPoint* pr)
{
	return pl->F < pr->F;
}



class CAstar
{
public:
	CAstar(int textureMap[][12])
	{
		for (int i = 0; i < 100; i++)
		{
			for (int j = 0; j < 100; j++)
			{
				m_textureMap[i][j] = 0;
			}
		}

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				m_textureMap[i][j] = textureMap[i][j];
			}
		}
	}

	CPoint* FindPath(CPoint* start, CPoint* end, bool IsIgnoreCorner)
	{
		m_listOpen.push_back(start);//����ʼ��ŵ������б���  

		while (m_listOpen.size())
		{
			CPoint* tempStart = getMinFPoint(); //��ȡFֵ��͵ĵ�  

			removeFromOpenList(tempStart);//�������ӿ����б���ɾ��  
			m_listClose.push_back(tempStart);//�������ŵ��ر��б���  

			std::vector<CPoint*> surroundPoints = SurrroundPoints(tempStart, IsIgnoreCorner);//��ȡFֵ��͵����ڵĵ�  

			tPointList::iterator _iter = surroundPoints.begin();

			//������Щ���ڵ�  
			for (_iter; _iter != surroundPoints.end(); ++_iter)
			{
				CPoint *point = *_iter;

				if (inOpenList(point->X, point->Y))//���������ڿ����б���  
					FoundPoint(tempStart, point);//���¼���Gֵ�����Gֵ��С������¸��ڵ㣬�������¼���Fֵ������ʲô������  
				else
					NotFoundPoint(tempStart, end, point);//���ڿ����б��У�����뿪���б�����Gֵ,Fֵ���趨���ڵ�  
			}
			if (inOpenList(end->X, end->Y))//Ŀ�����Ѿ��ڿ����б���  
			{
				//�����ڿ����б��еĸ��ڵ�  
				for (int i = 0; i < m_listOpen.size(); i++)
				{
					if (m_listOpen[i]->X == end->X && m_listOpen[i]->Y == end->Y)
					{
						return m_listOpen[i];
					}
				}
			}
		}
		return end;
	}

	bool CanReach(int x, int y)
	{
		return m_textureMap[x][y] == 0;
	}

	bool inCloseList(int x, int y)
	{
		CPoint* p = new CPoint(x, y);

		tPointList::iterator _iter = m_listClose.begin();
		for (_iter; _iter != m_listClose.end(); ++_iter)
		{
			CPoint* temp = *_iter;
			if (temp->X == p->X && temp->Y == p->Y)
				return true;
		}

		if (p)
		{
			delete p;
			p = NULL;
		}
		return false;
	}

	bool inOpenList(int x, int y)
	{
		CPoint *p = new CPoint(x, y);

		tPointList::iterator _iter = m_listOpen.begin();
		for (_iter; _iter != m_listOpen.end(); ++_iter)
		{
			CPoint* temp = *_iter;
			if (temp->X == p->X && temp->Y == p->Y)
				return true;
		}

		if (p)
		{
			delete p;
			p = NULL;
		}
		return false;
	}

	bool CanReach(CPoint* start, int x, int y, bool IsIgnoreCorner)
	{
		if (!CanReach(x, y) || inCloseList(x, y))
			return false;
		else
		{
			if ((abs(x - start->X) + abs(y - start->Y)) == 1)
				return true;
			else
			{
				if (CanReach(abs(x - 1), y) && CanReach(x, abs(y - 1)))
					return true;
				else
					return IsIgnoreCorner;//�Ƿ����ϰ���Խ��߲��ܿ�Խ�Ĺ���
			}
		}
	}

	std::vector<CPoint*> SurrroundPoints(CPoint* point, bool IsIgnoreCorner)
	{
		tPointList surroundPoints;

		for (int x = point->X - 1; x <= point->X + 1; x++)
			for (int y = point->Y - 1; y <= point->Y + 1; y++)
			{

			if (CanReach(point, x, y, IsIgnoreCorner))
			{

				CPoint *p = new CPoint(x, y);

				surroundPoints.push_back(p);
			}
			else
			{

			}
			}

		return surroundPoints;
	}



	CPoint* getMinFPoint()
	{

		tPointList tempList;

		for (int i = 0; i < (int)m_listOpen.size(); i++)
		{
			tempList.push_back(m_listOpen[i]);
		}
		sort(tempList.begin(), tempList.end(), CompF);


		if (tempList.size())
		{
			return tempList[0];
		}

	}

	void removeFromOpenList(CPoint* point)//ȫ���Ƴ�
	{
		tPointList::iterator _iter = m_listOpen.begin();
		for (_iter; _iter != m_listOpen.end(); ++_iter)
		{
			m_listOpen.erase(_iter);
			break;
		}
	}


	void FoundPoint(CPoint* tempStart, CPoint* point)
	{
		int G = CalcG(tempStart, point);
		if (G < point->G)
		{
			point->m_pParentPoint = tempStart;
			point->G = G;
			point->calcF();
		}
	}

	void NotFoundPoint(CPoint* tempStart, CPoint* end, CPoint* point)
	{
		point->m_pParentPoint = tempStart;


		point->G = CalcG(tempStart, point);
		point->H = CalcH(end, point);
		point->calcF();
		m_listOpen.push_back(point);
	}

	int CalcG(CPoint* start, CPoint* point)//�����ѡ�����񵽴�÷����Ƿ���и�С��Gֵ
	{
		int G = (abs(point->X - start->X) + abs(point->Y - start->Y)) == 1 ? STEP : OBLIQUE;
		int parentG = point->m_pParentPoint != NULL ? point->m_pParentPoint->G : 0;
		return G + parentG;
	}

	int CalcH(CPoint* end, CPoint* point)
	{
		int step = abs(point->X - end->X) + abs(point->Y - end->Y);
		return step * STEP;
	}

private:
	static const int STEP = 10;
	static const int OBLIQUE = 14;

	typedef std::vector<CPoint*> tPointList;

	tPointList m_listOpen;
	tPointList m_listClose;


	int m_textureMap[100][100];


};





int _tmain(int argc, _TCHAR* argv[])
{

	int array[12][12] = {
			{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
			{ 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1 },
			{ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1 },
			{ 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1 },
			{ 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1 },
			{ 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 },
			{ 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
			{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
	};
	CAstar *maze = new CAstar(array);
	CPoint *start = new CPoint(1, 1);
	CPoint *end = new CPoint(6, 10);
	CPoint* parent = maze->FindPath(start, end, true);


	while (parent != NULL)
	{
		cout << parent->X << "," << parent->Y << endl;
		parent = parent->m_pParentPoint;
	}
	system("pause");
	return 0;
}