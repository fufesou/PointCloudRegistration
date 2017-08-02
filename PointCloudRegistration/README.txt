2014/4/30
1. 工程先建成win32应用程序，方便就地写unit test。做完之后，再建DLL程序。
2. 算法简述
（1）配准主要分为粗配准和精细配准两部分。在工程中分别在CoarseRegistration和FineRegistration里。
（2）每部分由5小步组成：
	a. 寻找配准点云P的配准点集；					__  BaseSampler.h
	b. 寻找被配准点云Q上与P上配准点集的对应点集；	__  BaseCorrespondenceEstimation.h
	c. 去除无效配准点对；							__  BaseCorrespondenceRejector.h
	d. 计算旋转矩阵R和偏移向量T。					__  BaseTransformationEstimation.h
	e. 判断配准是否结束，如未结束，返回a。			__  BaseConvergenceCriteria.h
	
	

项目积累：
1. typedef使用建议(reference: expert C programming)
	a. 数组、结构、指针以及函数的组合类型
	b. 可移植类型
	c. 为后面的强制类型转换提供一个简单的名字
