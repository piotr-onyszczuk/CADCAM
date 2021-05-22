// ImportExportDoc.cpp : implementation of the CImportExportDoc class
//


#include "stdafx.h"
#include "ImportExportApp.h"

#include "ImportExportDoc.h"

#include <ImportExport/ImportExport.h>

#include <AISDialogs.h>
#include "res/resource.h"
#include <GeomLProp_SLProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <BRepAdaptor_Curve.hxx>


#ifdef _DEBUG
//#define new DEBUG_NEW  // by cascade
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CImportExportDoc

IMPLEMENT_DYNCREATE(CImportExportDoc, OCC_3dDoc)

BEGIN_MESSAGE_MAP(CImportExportDoc, OCC_3dDoc)
	//{{AFX_MSG_MAP(CImportExportDoc)
	ON_COMMAND(ID_FILE_IMPORT_BREP, OnFileImportBrep)
	ON_COMMAND(ID_FILE_IMPORT_IGES, OnFileImportIges)
	ON_COMMAND(ID_FILE_EXPORT_IGES, OnFileExportIges)
	ON_COMMAND(ID_FILE_IMPORT_STEP, OnFileImportStep)
	ON_COMMAND(ID_FILE_EXPORT_STEP, OnFileExportStep)
	ON_COMMAND(ID_FILE_EXPORT_VRML, OnFileExportVrml)
	ON_COMMAND(ID_FILE_EXPORT_STL, OnFileExportStl)
	ON_COMMAND(ID_BOX, OnBox)
	ON_COMMAND(ID_Cylinder, OnCylinder)
	ON_COMMAND(ID_OBJECT_REMOVE, OnObjectRemove)
	ON_COMMAND(ID_OBJECT_ERASE, OnObjectErase)
	ON_COMMAND(ID_OBJECT_DISPLAYALL, OnObjectDisplayall)
	//}}AFX_MSG_MAP

END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CImportExportDoc construction/destruction

CImportExportDoc::CImportExportDoc()
	: OCC_3dDoc(false)
{
	/*
		// TRIHEDRON
		Handle(AIS_Trihedron) aTrihedron;
		Handle(Geom_Axis2Placement) aTrihedronAxis=new Geom_Axis2Placement(gp::XOY());
		aTrihedron=new AIS_Trihedron(aTrihedronAxis);
		myAISContext->Display(aTrihedron);
	*/

	m_pcoloredshapeList = new CColoredShapes();
}

CImportExportDoc::~CImportExportDoc()
{
	if (m_pcoloredshapeList) delete m_pcoloredshapeList;
}


/////////////////////////////////////////////////////////////////////////////
// CSerializeDoc serialization

void CImportExportDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// Put the curent CColoredShape in the archive
		ar << m_pcoloredshapeList;
	}
	else
	{
		// Read from the archive the current CColoredShape
		ar >> m_pcoloredshapeList;

		// Display the new object
		m_pcoloredshapeList->Display(myAISContext);
	}
}


/*
void CImportExportDoc::OnWindowNew3d()
{
	((CImportExportApp*)AfxGetApp())->CreateView3D(this);
}
*/

//  nCmdShow could be :    ( default is SW_RESTORE ) 
// SW_HIDE   SW_SHOWNORMAL   SW_NORMAL   
// SW_SHOWMINIMIZED     SW_SHOWMAXIMIZED    
// SW_MAXIMIZE          SW_SHOWNOACTIVATE   
// SW_SHOW              SW_MINIMIZE         
// SW_SHOWMINNOACTIVE   SW_SHOWNA           
// SW_RESTORE           SW_SHOWDEFAULT      
// SW_MAX    

// use pViewClass = RUNTIME_CLASS( CImportExportView3D ) for 3D Views

void CImportExportDoc::ActivateFrame(CRuntimeClass* pViewClass, int nCmdShow)
{
	POSITION position = GetFirstViewPosition();
	while (position != (POSITION)NULL)
	{
		CView* pCurrentView = (CView*)GetNextView(position);
		if (pCurrentView->IsKindOf(pViewClass))
		{
			ASSERT_VALID(pCurrentView);
			CFrameWnd* pParentFrm = pCurrentView->GetParentFrame();
			ASSERT(pParentFrm != (CFrameWnd*)NULL);
			// simply make the frame window visible
			pParentFrm->ActivateFrame(nCmdShow);
		}
	}

}

/////////////////////////////////////////////////////////////////////////////
// CImportExportDoc diagnostics

#ifdef _DEBUG
void CImportExportDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CImportExportDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CImportExportDoc commands


void CImportExportDoc::OnFileImportBrep()
{
	Handle(TopTools_HSequenceOfShape) aSeqOfShape = CImportExport::ReadBREP();
	for (int i = 1; i <= aSeqOfShape->Length(); i++)
	{
		m_pcoloredshapeList->Add(Quantity_NOC_YELLOW, aSeqOfShape->Value(i));
		m_pcoloredshapeList->Display(myAISContext);
	}
	Fit();
}

#pragma region Nasze Modyfikacje
class Face
{
public:
	TopoDS_Face face;
	gp_Vec norm; //wektor normalny
	gp_Pnt center; //punkt na œcianie
	gp_Vec vec1, vec2; //wektory "rozpinaj¹ce" œcianê
	int wires; //liczba obwodów
	std::vector<gp_Pnt> edges; //lista krawêdzi (krawêdŸ reprezentuje jej œrodek)
	std::vector<int> neighbors; //lista s¹siadów indeksów w wektorze faces
	bool hasHole = false;
	bool isHole = false;
};

bool EdgesAreEqual(gp_Pnt e1, gp_Pnt e2)
{
	return e1.IsEqual(e2, 1e-3f);
}

std::vector<Face> faces;
bool IsConcave(int i, int j)
{
	gp_Vec v1 = faces[i].norm;
	auto v2 = gp_Vec(faces[i].center, faces[j].center);
	return v1.Dot(v2) < 0;
}

bool AllNeighborsConvex(int i, int except)
{
	bool ret = true;
	for (int j = 0; j < faces[i].neighbors.size(); ++j)
	{
		if (faces[i].neighbors[j] != except)
		{
			if (IsConcave(i, faces[i].neighbors[j]) || IsConcave(faces[i].neighbors[j], i))
			{
				ret = false;
			}
		}
	}
	return ret;
}

void SetAllNeighborsIsHole(int i, int except)
{
	for (int j = 0; j < faces[i].neighbors.size(); ++j)
	{
		if (faces[i].neighbors[j] != except)
		{
			if (!faces[faces[i].neighbors[j]].isHole)
			{
				faces[faces[i].neighbors[j]].isHole = true;
				SetAllNeighborsIsHole(faces[i].neighbors[j], except);
			}
		}
	}
}

void CImportExportDoc::OnFileImportIges()
{
	Handle(TopTools_HSequenceOfShape) aSeqOfShape = CImportExport::ReadIGES();
	faces.clear();
	myAISContext->RemoveAll(true);
	for (int i = 1; i <= aSeqOfShape->Length(); i++)
	{
		auto shape = aSeqOfShape->Value(i);
		TopExp_Explorer Ex1, Ex2, Ex3;
		for (Ex1.Init(shape, TopAbs_FACE); Ex1.More(); Ex1.Next()) {
			int d = 0;
			TopoDS_Shape face = Ex1.Current();
			TopoDS_Face ff = TopoDS::Face(face);

			BRepAdaptor_Surface aSurface(ff);
			Standard_Real u1, u2, v1, v2;
			u1 = aSurface.FirstUParameter();
			u2 = aSurface.LastUParameter();
			v1 = aSurface.FirstVParameter();
			v2 = aSurface.LastVParameter();

			gp_Pnt aCenterOfFace;
			gp_Vec aVec1, aVec2, aNormalOfFace;


			aSurface.D1((u1 + u2) / 2, (v1 + v2) / 2, aCenterOfFace, aVec1, aVec2);
			aNormalOfFace = aVec1 ^ aVec2;

			Face f;
			f.face = TopoDS::Face(face);
			f.norm = aNormalOfFace;
			f.center = aCenterOfFace;
			f.vec1 = aVec1;
			f.vec2 = aVec2;

			for (Ex2.Init(face, TopAbs_WIRE); Ex2.More(); Ex2.Next()) {
				TopoDS_Shape wire = Ex2.Current();
				d++;

				for (Ex3.Init(wire, TopAbs_EDGE); Ex3.More(); Ex3.Next()) {

					BRepAdaptor_Curve aCurve(TopoDS::Edge(Ex3.Current()));
					auto edge = aCurve.Value((aCurve.FirstParameter() + aCurve.LastParameter()) / 2.0f);
					f.edges.push_back(edge);

					for (int i = 0; i < faces.size(); ++i)
					{
						for (int j = 0; j < faces[i].edges.size(); ++j)
						{
							if (EdgesAreEqual(edge, faces[i].edges[j]))
							{
								faces[i].neighbors.push_back(faces.size());
								f.neighbors.push_back(i);
							}
						}
					}
				}
			}
			f.wires = d;
			faces.push_back(f);
		}
	}

	for (int i = 0; i < faces.size(); ++i)
	{
		if (faces[i].wires > 1 && !faces[i].isHole)
		{
			for (int j = 0; j < faces[i].neighbors.size(); ++j)
			{
				if (IsConcave(i, faces[i].neighbors[j]))
				{
					if (AllNeighborsConvex(faces[i].neighbors[j], i))
					{
						faces[i].hasHole = true;
						faces[faces[i].neighbors[j]].isHole = true;
						SetAllNeighborsIsHole(faces[i].neighbors[j], i);
					}
				}
			}
		}
	}

	for (int i = 0; i < faces.size(); ++i)
	{
		Handle(AIS_Shape) aface = new AIS_Shape(faces[i].face);
		if (faces[i].isHole == true)
		{
			myAISContext->SetColor(aface, Quantity_NOC_GREEN, Standard_False);
		}
		else
		{
			if (faces[i].hasHole == true)
				myAISContext->SetColor(aface, Quantity_NOC_RED, Standard_False);
			else
				myAISContext->SetColor(aface, Quantity_NOC_YELLOW, Standard_False);
		}
		myAISContext->SetMaterial(aface, Graphic3d_NOM_PLASTIC, Standard_False);
		myAISContext->SetTransparency(aface, 0.0f, Standard_False);
		myAISContext->Display(aface, 1, 0, Standard_False);
	}

	Fit();
}
#pragma endregion


void CImportExportDoc::OnFileExportIges()
{
	CImportExport::SaveIGES(myAISContext);
}

void CImportExportDoc::OnFileImportStep()
{
	Handle(TopTools_HSequenceOfShape) aSeqOfShape = CImportExport::ReadSTEP();
	for (int i = 1; i <= aSeqOfShape->Length(); i++)
	{
		m_pcoloredshapeList->Add(Quantity_NOC_YELLOW, aSeqOfShape->Value(i));
		m_pcoloredshapeList->Display(myAISContext);
	}
	Fit();
}

void CImportExportDoc::OnFileExportStep()
{
	CImportExport::SaveSTEP(myAISContext);
}


void CImportExportDoc::OnFileExportVrml()
{
	CImportExport::SaveVRML(myAISContext);
}

void CImportExportDoc::OnFileExportStl()
{
	CImportExport::SaveSTL(myAISContext);
}

void  CImportExportDoc::Popup(const Standard_Integer  x,
	const Standard_Integer  y,
	const Handle(V3d_View)& aView)
{
	Standard_Integer PopupMenuNumber = 0;
	myAISContext->InitSelected();
	if (myAISContext->MoreSelected())
		PopupMenuNumber = 1;

	CMenu menu;
	VERIFY(menu.LoadMenu(IDR_Popup3D));
	CMenu* pPopup = menu.GetSubMenu(PopupMenuNumber);

	ASSERT(pPopup != NULL);
	if (PopupMenuNumber == 1) // more than 1 object.
	{
		bool OneOrMoreInShading = false;
		for (myAISContext->InitSelected(); myAISContext->MoreSelected(); myAISContext->NextSelected())
			if (myAISContext->IsDisplayed(myAISContext->SelectedInteractive(), 1)) OneOrMoreInShading = true;
		if (!OneOrMoreInShading)
			pPopup->EnableMenuItem(5, MF_BYPOSITION | MF_DISABLED | MF_GRAYED);
	}

	POINT winCoord = { x , y };
	Handle(WNT_Window) aWNTWindow =
		Handle(WNT_Window)::DownCast(aView->Window());
	ClientToScreen((HWND)(aWNTWindow->HWindow()), &winCoord);
	pPopup->TrackPopupMenu(TPM_LEFTALIGN | TPM_RIGHTBUTTON, winCoord.x, winCoord.y,
		AfxGetMainWnd());


}

void CImportExportDoc::OnBox()
{
	AIS_ListOfInteractive aList;
	myAISContext->DisplayedObjects(aList);
	AIS_ListIteratorOfListOfInteractive aListIterator;
	for (aListIterator.Initialize(aList); aListIterator.More(); aListIterator.Next()) {
		myAISContext->Remove(aListIterator.Value(), Standard_False);
	}

	BRepPrimAPI_MakeBox B(200., 150., 100.);

	m_pcoloredshapeList->Add(Quantity_NOC_YELLOW, B.Shape());

	m_pcoloredshapeList->Display(myAISContext);
	Fit();

	// document has been modified
	SetModifiedFlag(TRUE);
}

void CImportExportDoc::OnCylinder()
{
	AIS_ListOfInteractive aList;
	myAISContext->DisplayedObjects(aList);
	AIS_ListIteratorOfListOfInteractive aListIterator;
	for (aListIterator.Initialize(aList); aListIterator.More(); aListIterator.Next()) {
		myAISContext->Remove(aListIterator.Value(), Standard_False);
	}

	BRepPrimAPI_MakeCylinder C(50., 200.);

	m_pcoloredshapeList->Add(Quantity_NOC_GREEN, C.Shape());

	m_pcoloredshapeList->Display(myAISContext);
	Fit();

	// document has been modified
	SetModifiedFlag(TRUE);
}
void CImportExportDoc::OnObjectRemove()

{
	for (GetAISContext()->InitSelected(); GetAISContext()->MoreSelected(); GetAISContext()->NextSelected()) {
		Handle(AIS_Shape) aShape = Handle(AIS_Shape)::DownCast(GetAISContext()->SelectedInteractive());
		if (!aShape.IsNull()) {
			m_pcoloredshapeList->Remove(aShape->Shape());
		}
	}
	OCC_3dBaseDoc::OnObjectRemove();
}

void CImportExportDoc::OnObjectErase()

{
	for (GetAISContext()->InitSelected(); GetAISContext()->MoreSelected(); GetAISContext()->NextSelected()) {
		Handle(AIS_Shape) aShape = Handle(AIS_Shape)::DownCast(GetAISContext()->SelectedInteractive());
		if (!aShape.IsNull()) {
			m_pcoloredshapeList->Remove(aShape->Shape());
		}
	}
	OCC_3dBaseDoc::OnObjectErase();
}

void CImportExportDoc::OnObjectDisplayall()

{
	OCC_3dBaseDoc::OnObjectDisplayall();
}