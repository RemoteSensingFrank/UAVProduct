//
// Created by wuwei on 17-7-30.
//
#include "UAVAuxiliary.h"

void AdjacencyMatrixToSVG
        (
                const size_t NbImages,
                const openMVG::Pair_Set & corresponding_indexes,
                const std::string & sOutName
        )
{
    using namespace svg;
    if (!corresponding_indexes.empty())
    {
        const float scaleFactor = 5.0f;
        svgDrawer svgStream((NbImages+3)*5, (NbImages+3)*5);
        // List possible pairs
        for (size_t I = 0; I < NbImages; ++I)
        {
            for (size_t J = 0; J < NbImages; ++J)
            {
                // If the pair have matches display a blue boxes at I,J position.
                const auto iterSearch = corresponding_indexes.find(std::make_pair(I,J));
                if (iterSearch != corresponding_indexes.end())
                {
                    svgStream.drawSquare(J*scaleFactor, I*scaleFactor, scaleFactor/2.0f,
                                         svgStyle().fill("blue").noStroke());
                }
            }
        }
        // Display axes with 0 -> NbImages annotation : _|
        std::ostringstream osNbImages;
        osNbImages << NbImages;
        svgStream.drawText((NbImages+1)*scaleFactor, scaleFactor, scaleFactor, "0", "black");
        svgStream.drawText((NbImages+1)*scaleFactor,
                           (NbImages)*scaleFactor - scaleFactor, scaleFactor, osNbImages.str(), "black");
        svgStream.drawLine((NbImages+1)*scaleFactor, 2*scaleFactor,
                           (NbImages+1)*scaleFactor, (NbImages)*scaleFactor - 2*scaleFactor,
                           svgStyle().stroke("black", 1.0));

        svgStream.drawText(scaleFactor, (NbImages+1)*scaleFactor, scaleFactor, "0", "black");
        svgStream.drawText((NbImages)*scaleFactor - scaleFactor,
                           (NbImages+1)*scaleFactor, scaleFactor, osNbImages.str(), "black");
        svgStream.drawLine(2*scaleFactor, (NbImages+1)*scaleFactor,
                           (NbImages)*scaleFactor - 2*scaleFactor, (NbImages+1)*scaleFactor,
                           svgStyle().stroke("black", 1.0));

        std::ofstream svgFileStream(sOutName.c_str());
        svgFileStream << svgStream.closeSvgFile().str();
    }
}

//空间后方交会残差计算
struct ResectionResidual
{
    /*
    * X, Y, Z, x, y 分别为观测值，f为焦距
    */
    ResectionResidual(double X, double Y, double Z, double x, double y, double f)
        :_X(X), _Y(Y), _Z(Z), _x(x), _y(y), _f(f){}

    /*
    * pBackCrossParameters：-2分别为Xs、Ys、Zs,3-5分别为Phi、Omega、Kappa
    */
    template <typename T>
    bool operator () (const T * const pBackCrossParameters, T* residual) const
    {
        T dXs = pBackCrossParameters[0];
        T dYs = pBackCrossParameters[1];
        T dZs = pBackCrossParameters[2];
        T dPhi = pBackCrossParameters[3];
        T dOmega = pBackCrossParameters[4];
        T dKappa = pBackCrossParameters[5];

        T a1  = cos(dPhi)*cos(dKappa) - sin(dPhi)*sin(dOmega)*sin(dKappa);
        T a2  = -cos(dPhi)*sin(dKappa) - sin(dPhi)*sin(dOmega)*cos(dKappa);
        T a3  = -sin(dPhi)*cos(dOmega);
        T b1 = cos(dOmega)*sin(dKappa);
        T b2 = cos(dOmega)*cos(dKappa);
        T b3 = -sin(dOmega);
        T c1 = sin(dPhi)*cos(dKappa) + cos(dPhi)*sin(dOmega)*sin(dKappa);
        T c2 = -sin(dPhi)*sin(dKappa) + cos(dPhi)*sin(dOmega)*cos(dKappa);
        T c3 = cos(dPhi)*cos(dOmega);

        // 有两个残差
        residual[0]= T(_x) +T(_f) * T( (a1*(_X-dXs) + b1*(_Y-dYs) + c1*(_Z-dZs)) / ((a3*(_X-dXs) + b3*(_Y-dYs) + c3*(_Z-dZs))));
        residual[1]= T(_y) +T(_f) * T( (a2*(_X-dXs) + b2*(_Y-dYs) + c2*(_Z-dZs)) / ((a3*(_X-dXs) + b3*(_Y-dYs) + c3*(_Z-dZs))));

        return true;
    }

private:
    const double _X;
    const double _Y;
    const double _Z;
    const double _x;
    const double _y;
    const double _f;
};

void Resection(double* gcps,int gcpnum,double* param)
{
    
}
