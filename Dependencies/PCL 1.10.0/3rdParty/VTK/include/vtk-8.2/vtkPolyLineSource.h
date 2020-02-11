/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPolyLineSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkPolyLineSource
 * @brief   create a poly line from a list of input points
 *
 * vtkPolyLineSource is a source object that creates a poly line from
 * user-specified points. The output is a vtkPolyLine.
*/

#ifndef vtkPolyLineSource_h
#define vtkPolyLineSource_h

#include "vtkFiltersSourcesModule.h" // For export macro
#include "vtkPolyDataAlgorithm.h"

class vtkPoints;

class VTKFILTERSSOURCES_EXPORT vtkPolyLineSource : public vtkPolyDataAlgorithm
{
public:
  static vtkPolyLineSource* New();
  vtkTypeMacro(vtkPolyLineSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  //@{
  /**
   * Set the number of points in the poly line.
   */
  void SetNumberOfPoints(vtkIdType numPoints);
  vtkIdType GetNumberOfPoints();
  //@}

  /**
   * Resize while preserving data.
   */
  void Resize(vtkIdType numPoints);

  /**
   * Set a point location.
   */
  void SetPoint(vtkIdType id, double x, double y, double z);

  //@{
  /**
   * Get the points.
   */
  void SetPoints(vtkPoints* points);
  vtkGetObjectMacro(Points, vtkPoints);
  //@}

  //@{
  /**
   * Set whether to close the poly line by connecting the last and first points.
   */
  vtkSetMacro(Closed, vtkTypeBool);
  vtkGetMacro(Closed, vtkTypeBool);
  vtkBooleanMacro(Closed, vtkTypeBool);
  //@}

protected:
  vtkPolyLineSource();
  ~vtkPolyLineSource() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector *) override;

  vtkPoints* Points;

  vtkTypeBool Closed;

private:
  vtkPolyLineSource(const vtkPolyLineSource&) = delete;
  void operator=(const vtkPolyLineSource&) = delete;
};

#endif
