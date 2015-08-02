#ifndef _EDITTRIMESH_H_
#define _EDITTRIMESH_H_

#include "vector3.h"
class Matrix4x3;

class EditTriMesh
{
public:
	//Vertex类用于保存顶点信息
	class Vertex
	{
	public:
		Vertex() 
		{
			setDefaults();
		}
		void setDefaults();
		//3D坐标
		Vector3 p;

		//顶点级纹理映射坐标,可能失效。
		float u, v;
		//顶点级表面法向量，同样可能失效
		Vector3 normal;
		//工具变量
		int mark;
	};

	//类Tri用于保存三角形信息
	class Tri
	{
	public:
		
		Tri() { setDefaults(); }
		void setDefaults();

		//面顶点
		struct Vert
		{
			int index;   //顶点列表的索引
			float u, v;  //纹理坐标
		};

		Vert v[3];
		//表面法向量
		Vector3 normal;
		//属于网格的哪部分
		int part;
		//材质列表索引
		int material;
		//工具变量
		int mark;
		//判断是否为退化三角形---同一顶点在一个三角形中使用多次
		bool isDegenerate() const;

		//返回顶点索引，为使用该顶点则返回-1
		int findVertex(int vertexIndex) const;
	};

	//材质类，保存材质信息

	class Material
	{
	public:

		Material() { setDefaults(); }
		void setDefaults();
		char diffuseTextureName[256];

		//工具变量
		int mark;
	};

	//控制优化的选项
	class OptimationParameters
	{
	public:
		OptimationParameters(){ setDefaults(); }
		void setDefaults();

		//判断两个顶点是否重合的容差
		float coincidentVertexTolerance;

		float cosOfEdgeAngleTolerance;
		void setEdgeAngleToleranceInDegrees(float degrees);
	};

	EditTriMesh();
	EditTriMesh(const EditTriMesh& x);

	~EditTriMesh();

	EditTriMesh& operator=(const EditTriMesh& rhs);

	//存取网格数据

	int vertexCount() const { return vCount; }
	int triCount() const { return tCount; }
	int materialCount() const { return mCount; }

	Vertex& vertex(int vertexIndex);
	const Vertex& vertex(int vertexIndex) const;
	
	Tri& tri(int triIndex);
	const Tri& tri(int triIndex) const;
	
	Material& material(int materialIndex);
	const Material& material(int materialIndex) const;

	//重置网格为空
	void empty();

	//设置列表大小
	void setVertexCount(int vc);
	void setTriCount(int tc);
	void setMaterialCount(int mc);

	//添加三角形\顶点\材质，返回新加入元素的索引

	int addTri();
	int addTri(const Tri& t);

	int addVertex();
	int addVertex(const Vertex& v);
	int dupVertex(int srcVertexIndex);

	int addMaterial(const Material& m);

	//同时设置所有mark变量
	void markAllVertices(int mark);
	void markAllTris(int mark);
	void markAllMaterials(int mark);

	//删除操作

	void deleteVertex(int vertexIndex);
	void deleteTri(int triIndex);
	void deleteMaterial(int materialIndex);
	void deleteUnusedMaterials();
	void deleteMarkedTris(int mark);
	void deleteDegenerateTris();

	//逐个分离所有面，产生一个新的顶点列表，每个顶点只用于一个三角形，同时删除未用的点
	void detachAllFaces();

	//变换所有顶点
	void transformVertices(const Matrix4x3& m);


	//计算三角形的表面法向量
	void computeOneTriNormal(int triIndex);
	void computeOneTriNormal(Tri& t);
	void computeOneTriNormals();
	
	//计算顶点法向量，自动计算三角形法向量
	void computeVertexNormals();

	//根据使用情况重新排列顶点列表，增强Cache的性能，删除未使用的顶点
	void optimizeVertexOrder(bool removeUnusedVertices);

	//根据材质为三角形排序
	void sortTrisByMaterial();

	//焊接顶点
	void weldVertices(const OptimationParameters* opt);

	//确保UV值正确，如果需要可能会复制顶点
	void copyUvsIntoVertices();
	void optimizeForRedering();

	//导入到处S3D模型
	bool importS3d(const char* filename, char* returnErrMsg);
	bool exportS3d(const char* filename, char* returnErrMsg);
	
	//调试用
	void validityCheck();
	bool validityCheck(char* returnErrMsg);




private:
	int vAlloc;
	int vCount;
	Vertex* vList;
	int tAlloc;
	int tCount;
	Tri* tList;
	int mCount;
	Material * mList;

	void construct();

};


#endif // !_EDITTRIMESH_H_
