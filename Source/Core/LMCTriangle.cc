#include "LMCTriangle.hh"

namespace locust
{
    const LMCTriangle gZeroTriangle(LMCThreeVector::sInvalid,LMCThreeVector::sInvalid,LMCThreeVector::sInvalid);
    const LMCTriangle gInvalidTriangle(LMCThreeVector::sZero,LMCThreeVector::sZero,LMCThreeVector::sZero);

    LMCTriangle::LMCTriangle(const LMCTriangle& aTriangle)
    {
        for (int i=0; i<3; ++i)
        {
            fVertices[i]=aTriangle.GetVertex(i);
        }
    }

    LMCTriangle::LMCTriangle(const LMCThreeVector& vertex0, const LMCThreeVector& vertex1, const LMCThreeVector& vertex2)
    {
        fVertices[0]=vertex0;
        fVertices[1]=vertex1;
        fVertices[2]=vertex2;
    }

    LMCTriangle::LMCTriangle(const double vertex0[3], const double vertex1[3], const double vertex2[3])
    {
        fVertices[0]=vertex0;
        fVertices[1]=vertex1;
        fVertices[2]=vertex2;
    }

    LMCTriangle::~LMCTriangle(){}

    LMCTriangle& LMCTriangle::operator=(const LMCTriangle& aTriangle)
    {
        for (int i=0; i<3; ++i)
        {
            fVertices[i]=aTriangle.GetVertex(i);
        }
        return *this;
    }
            
    LMCThreeVector& LMCTriangle::operator[](int vertexIndex)
    {
        return fVertices[vertexIndex];
    }

    const LMCThreeVector& LMCTriangle::operator[](int vertexIndex) const
    {
        return fVertices[vertexIndex];
    }

    bool LMCTriangle::operator==(const LMCTriangle& aTriangle) const
    {
        return ((aTriangle.GetVertex(0)==fVertices.at(0)) && (aTriangle.GetVertex(1)==fVertices.at(1)) && (aTriangle.GetVertex(2)==fVertices.at(2)));
    }

    bool LMCTriangle::operator!=(const LMCTriangle& aTriangle) const
    {
        return ((aTriangle.GetVertex(0)!=fVertices.at(0)) && (aTriangle.GetVertex(1)!=fVertices.at(1)) && (aTriangle.GetVertex(2)!=fVertices.at(2)));
    }

    LMCTriangle& LMCTriangle::operator+=(const LMCTriangle& aTriangle) 
    {
        for (int i=0; i<3; ++i)
        {
            fVertices.at(i)+=aTriangle.GetVertex(i);
        }
        return *this;
    }

    LMCTriangle& LMCTriangle::operator+=(const LMCThreeVector& aVector) 
    {
        for (int i=0; i<3; ++i)
        {
            fVertices.at(i)+=aVector;
        }
        return *this;
    }

    LMCTriangle& LMCTriangle::SetVertex(int vertexIndex,const LMCThreeVector& vertex)
    {
        fVertices[vertexIndex]=vertex;
        return *this;
    }

    const LMCThreeVector& LMCTriangle::GetVertex(int vertexIndex) const
    {
        return fVertices.at(vertexIndex);
    }
            
    const std::array<LMCThreeVector,3>& LMCTriangle::GetVertices(int vertexIndex) const
    {
        return fVertices;
    }

    const LMCThreeVector LMCTriangle::GetMidPoint01() const
    {
        LMCThreeVector point((fVertices.at(0)+fVertices.at(1))/2.0);
        return point;
    }

    const LMCThreeVector LMCTriangle::GetMidPoint02() const
    {
        LMCThreeVector point((fVertices.at(0)+fVertices.at(2))/2.0);
        return point;
    }

    const LMCThreeVector LMCTriangle::GetMidPoint12() const
    {
        LMCThreeVector point((fVertices.at(1)+fVertices.at(2))/2.0);
        return point;
    }

    const void LMCTriangle::GetMidPoints(LMCThreeVector& midPoint01,LMCThreeVector& midPoint02,LMCThreeVector& midPoint12) const
    {
        midPoint01=GetMidPoint01();
        midPoint02=GetMidPoint02();
        midPoint12=GetMidPoint12();
        return;
    }

    double LMCTriangle::GetArea() const
    {
        LMCThreeVector line01=fVertices.at(0)-fVertices.at(1);
        LMCThreeVector line02=fVertices.at(0)-fVertices.at(2);
        return (line01.Cross(line02)).Magnitude()/2.0;
    }

    const LMCThreeVector LMCTriangle::GetNormal() const
    {
        LMCThreeVector line01=fVertices.at(0)-fVertices.at(1);
        LMCThreeVector line02=fVertices.at(0)-fVertices.at(2);
        // need to conform the direction, there are two possibilities
        return (line01.Cross(line02)).Unit();
    }
}
