/*
 *  CST_Path.h
 *  CST_G1
 *
 *  Created by Chris Tharpe on 8/29/11.
 *  Copyright 2011 Christopher S. Tharpe. All rights reserved.
 *
 */
#ifndef CST_PATH

#define CST_PATH

#include "CST_GameObject.h"
#include "CST_SlidingArray.h"

#define CST_NUMBER_OF_TERMS  (8)

#define CST_MIN_DISTANCE  (9.0)

//#define CST_MAX_POINTS  (64)
#define CST_MAX_POINTS (16)
//#define CST_MAX_POINTS (640)


typedef struct CST_Motion
{
    CST_Motion()
    {
        Init();
    }
    
    ~CST_Motion(){}
    
    void Init(void)
    {
        Clear();
    };
    
    void Clear(void)
    {
        m_fVx = 0.0;
        m_fVy = 0.0;
        m_fVz = 0.0;
        m_fi = 0.0;
        m_fj = 0.0;
        m_fk = 0.0;
        m_fAngle = 0.0;  //m_fAngle is actually angular velocity, in degrees/second
        m_fdeltaTime = 1.0;  //set default to 1.0, if it's 0.0, no motion will take place
        m_fSinAngle = 0.0;
        m_fSpeed = 0.0;
        m_ft = 0;
        m_bPathLimitReached = false;
    };
    
    void Normalize(void)
    {
        GLfloat magnitude = sqrt(m_fi * m_fi + m_fj * m_fj + m_fk * m_fk);
        if(magnitude)
        {
            m_fi /= magnitude;
            m_fj /= magnitude;
            m_fk /= magnitude;
        }
    };
    
    bool    m_bPathLimitReached;
    GLfloat m_fVx, m_fVy, m_fVz;
    GLfloat m_fi, m_fj, m_fk;
    GLfloat m_fAngle, m_fdeltaTime;
    GLfloat m_fSinAngle;
    GLfloat m_fSpeed;
    GLfloat m_ft;  //this is not time, this is parameter for use in Path objects
    
}_CST_Motion;


class CST_Path : public CST_GameObject
	{
	public:
		CST_Path();
		virtual ~CST_Path();
		
		void Init();
		
		//parametric equations, give you (x,y,z) from t
		GLfloat FX(GLfloat t = 0);
		GLfloat FY(GLfloat t = 0);
		GLfloat FZ(GLfloat t = 0);
		void FParametric(GLfloat t = 0, GLfloat *pResult = 0);
        void GetStartPoint(GLfloat *pResult = 0);
        void GetEndPoint(GLfloat *pResult = 0);
		void UpdatePosition(CST_Motion *pMotion = 0, CST_Motion *pMotionAbsolute = 0, GLfloat *pDirectionVector = 0);
		
		//set path paramters
		bool SetA(int index = 0, GLfloat value = 0.0);
		bool SetB(int index = 0, GLfloat value = 0.0);
		bool SetC(int index = 0, GLfloat value = 0.0);
        
        GLfloat GetA(int index = 0);
        GLfloat GetB(int index = 0);
        GLfloat GetC(int index = 0);
		
		void SetRepeat(bool repeat = true){m_bRepeat = repeat;};
		void SetUpperLimit(GLfloat upper = 2.0 * CST_PI){m_fUpperLimit = upper;};
		void SetLowerLimit(GLfloat lower = 0.0){m_fLowerLimit = lower;};
        void SetMFDeltaTSign(void);
        
        GLfloat GetUpperLimit(void){return m_fUpperLimit;};
        GLfloat GetLowerLimit(void){return m_fLowerLimit;};
        
        void SetDrawPath(bool b_draw = false){m_bDrawPath = b_draw;};
        bool GetDrawPath(void){return m_bDrawPath;};
		
		void ClearTerms(void);
        
        virtual void CreateVertices(void);
        
        void SetRotatePath(bool rp = false){m_bRotatePath = rp;};
        
        int GetMaxPoints(void){return m_nMaxPointsInDrawnPath;};
        int GetMaxIndices(void){return (2 * (m_nMaxPointsInDrawnPath - 1));};
        void SetMaxPointsForDrawnPath(int max = CST_MAX_POINTS);
        
        void ResetSlidingArray(void);
        
        void SetPathCentroid(GLfloat *pPoint = 0);
        GLfloat* GetPointerToPathCentroid(void);
        
        void SetupPathRotation(GLfloat *pCentroid = 0, CST_Motion *pMotion = 0);
        
        void SetUpperLimitOffScreen(GLfloat safety_distance = 0.0);
        void SetLowerLimitOffScreen(GLfloat safety_distance = 0.0);
        void SetUpperAndLowerLimitOffScreen(GLfloat safety_distance = 0.0);        
		
	protected:
		GLfloat m_fdeltaT;
		GLfloat m_fRadius;
		bool    m_bRepeat;  //determines whether path repeats, or just goes through once (e.g. circle v. line segment)
        bool    m_bRotatePath;  //determines whether path rotated
		GLfloat m_fUpperLimit;  //upper limit of parametric variable
		GLfloat m_fLowerLimit;  //lower limit of parametric variable
        GLfloat m_fdTSign;
        GLfloat m_pPathCentroid[CST_DIMENSION];
        CST_SlidingArray m_cSlidingArray;
        int     m_nMaxPointsInDrawnPath;
		
		//could obviously use 2-d array (e.g, A[3][CST_NUMBER_OF_TERMS]) instead of A, B, C; but
		//easier to keep track of like this:
		GLfloat A[CST_NUMBER_OF_TERMS], B[CST_NUMBER_OF_TERMS], C[CST_NUMBER_OF_TERMS];
		
		bool LimitParameter(CST_Motion *pMotion = 0);
        
        bool m_bDrawPath;
        bool m_bFirstTime;
        
        void AddPointToBeDrawn(GLfloat *pPoint = 0);
        void AddPointToSlidingArray(GLfloat *pPoint = 0);
        
        void SetParameterLimitOffScreen(GLfloat *pParameter = 0, GLfloat sign = 1.0, GLfloat safety_distance = 0.0);

	};

#endif //CST_PATH
