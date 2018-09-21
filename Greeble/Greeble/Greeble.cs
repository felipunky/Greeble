using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Greeble
{

    public class Greeble : GH_Component
    {

        public Greeble() : base("Greeble", "Greeble", "Turns points into a Greebled surface", "Surface", "Util")
        {



        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {

            pManager.AddPointParameter("PointsIn", "Points", "Points to compute the mesh and commence the Greeble Algorithm", GH_ParamAccess.list);
            pManager.AddIntegerParameter("SubDivideIterations", "SubDivideIter", "Number of times to divide each of the mesh faces", GH_ParamAccess.item);
            pManager.AddNumberParameter("Treshold", "Treshold", "Treshold for the mesh's faces subdivision", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Sierpinsky", "Sierpinsky", "Wheter or not to compute the Greeble Algorithm with a Sierpinsky subdivision", GH_ParamAccess.item);
            pManager.AddIntegerParameter("IterationNumber", "Iter", "Number of times we want to repeat the algorithm", GH_ParamAccess.item);

        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {

            pManager.AddGeometryParameter("GreebledSurface", "Greebled", "The Greeble Algorithm output", GH_ParamAccess.tree);

        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {

            checked
            {

                var points = new List<Point3d>();
                DA.GetDataList(0, points);

                int subDivideIter = 0;
                DA.GetData(1, ref subDivideIter);

                double treshold = 0.0;
                DA.GetData(2, ref treshold);

                bool sierpinsky = false;
                DA.GetData(3, ref sierpinsky);

                int iter = 0;
                DA.GetData(4, ref iter);

                var extrusionFaces = new List<Curve>();
                var extrusions = FirstIteration(points, out extrusionFaces);

                var extrusionFacesIteration = new List<Curve>[iter + 2];
                var extrusionIteration = new List<Extrusion>[iter + 2];

                var treeOut = new DataTree<Extrusion>();

                extrusionIteration[0] = Iterations(extrusions, extrusionFaces, out extrusionFacesIteration[0], subDivideIter, treshold, sierpinsky);

                for (int i = 0; i < extrusions.Count; ++i)
                {

                    treeOut.Add(extrusions[i], new GH_Path(0));

                }

                for (int i = 0; i < iter + 1; ++i)
                {

                    treshold *= 0.5;
                    extrusionIteration[i + 1] = Iterations(extrusionIteration[i], extrusionFacesIteration[i], out extrusionFacesIteration[i + 1], subDivideIter, treshold, sierpinsky);

                    for (int j = 0; j < extrusionIteration[i].Count; ++j)
                    {

                        treeOut.Add(extrusionIteration[i][j], new GH_Path(i));

                    }

                }

                DA.SetDataTree(0, treeOut);

            }

        }

        public override Guid ComponentGuid
        {

            get { return new Guid("18D1DC2B-8104-4DE4-B384-48068B6195E3"); }

        }

        private List<Extrusion> Iterations(List<Extrusion> extrusions, List<Curve> extrusionFaces, out List<Curve> extrusionFacesTree, int subDivideIter, double treshold, bool sierpinksy)
        {

            // We finished our first iteration, now to set up the loopable part.
            var polyLines = new Polyline[extrusions.Count];
            var polyLinePoints = new List<Point3d>[extrusions.Count];
            var meshArray = new Mesh[extrusions.Count];

            for (int i = 0; i < extrusions.Count; ++i)
            {

                extrusionFaces[i].TryGetPolyline(out polyLines[i]);
                polyLinePoints[i] = (polyLines[i].ToList());
                polyLinePoints[i].RemoveAt(0);
                meshArray[i] = DelaunayPoints(polyLinePoints[i]);

                // Subdivide and Triangulate mesh.
                meshArray[i] = Triangulate(meshArray[i]);
                for (int j = 0; j < subDivideIter; ++j)
                {

                    Subdivide(meshArray[i], treshold, sierpinksy);

                }

            }

            var extrusionFacesOne = new List<Curve>[meshArray.Length];
            var extrusionsOne = new List<Extrusion>[meshArray.Length];
            var extrusionTree = new List<Extrusion>();
            extrusionFacesTree = new List<Curve>();

            for (int i = 0; i < meshArray.Length; ++i)
            {

                extrusionsOne[i] = (PointsToExtrusion(meshArray[i], out extrusionFacesOne[i]));

                var path = new GH_Path(i + 1);


                for (int j = 0; j < extrusionsOne[i].Count; ++j)
                {

                    extrusionTree.Add(extrusionsOne[i][j]);
                    extrusionFacesTree.Add(extrusionFacesOne[i][j]);

                }

            }

            return extrusionTree;

        }

        private List<Extrusion> FirstIteration(List<Point3d> points, out List<Curve> extrusionFaces)
        {

            // Our first iteration.
            var rhinoMesh = DelaunayPoints(points);

            extrusionFaces = new List<Curve>();

            return PointsToExtrusion(rhinoMesh, out extrusionFaces);

        }

        private List<Extrusion> PointsToExtrusion(Mesh rhinoMesh, out List<Curve> extrusionFaces)
        {

            var ghMesh = GHMesh(rhinoMesh);

            var faceMesh = new List<object>();
            Point3d[] rhinoVerMesh = DeconstructMesh(ghMesh, out faceMesh);

            int[] faceA = new int[rhinoMesh.Faces.Count];
            int[] faceB = new int[rhinoMesh.Faces.Count];
            int[] faceC = new int[rhinoMesh.Faces.Count];
            int[] faceD = MeshFaces(rhinoMesh, out faceA, out faceB, out faceC);

            var listA = new List<Point3d>();
            var listB = new List<Point3d>();
            var listC = new List<Point3d>();
            var listD = new List<Point3d>();
            for (int i = 0; i < faceA.Length; ++i)
            {

                listA.Add(ListItem(rhinoVerMesh.ToList(), faceA[i]));
                listB.Add(ListItem(rhinoVerMesh.ToList(), faceB[i]));
                listC.Add(ListItem(rhinoVerMesh.ToList(), faceC[i]));
                listD.Add(ListItem(rhinoVerMesh.ToList(), faceD[i]));

            }

            extrusionFaces = new List<Curve>();

            var extrusions = SurfaceAndExtrude(listA, listB, listC, listD, out extrusionFaces).ToList();

            return extrusions;

        }

        private List<Curve> ExtrusionFaces(List<Extrusion> extrusions)
        {

            var extrusionFaces = new Curve[extrusions.Count];

            for (int i = 0; i < extrusions.Count; ++i)
            {

                extrusionFaces[i] = extrusions[i].GetWireframe()[1];

            }

            return extrusionFaces.ToList();

        }

        private Extrusion[] SurfaceAndExtrude(List<Point3d> listA, List<Point3d> listB, List<Point3d> listC, List<Point3d> listD, out List<Curve> extrusionFacesOut)
        {

            var doc = new Grasshopper.Kernel.GH_Document();
            var surFourPoints = new SurfaceComponents.SurfaceComponents.Component_4PointSurface();
            var surFourParamsA = surFourPoints.Params.Input[0] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            surFourParamsA.PersistentData.ClearData();
            var surFourParamsB = surFourPoints.Params.Input[1] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            surFourParamsB.PersistentData.ClearData();
            var surFourParamsC = surFourPoints.Params.Input[2] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            surFourParamsC.PersistentData.ClearData();
            var surFourParamsD = surFourPoints.Params.Input[3] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            surFourParamsD.PersistentData.ClearData();
            for (int i = 0; i < listA.Count; ++i)
            {

                surFourParamsA.PersistentData.Append(new GH_Point(listA[i]));
                surFourParamsB.PersistentData.Append(new GH_Point(listB[i]));
                surFourParamsC.PersistentData.Append(new GH_Point(listC[i]));
                surFourParamsD.PersistentData.Append(new GH_Point(listD[i]));

            }

            var surfaceFourPoints = new List<object>();
            var surfaces = new Grasshopper.Kernel.Types.GH_Surface[listA.Count];
            var breps = new Rhino.Geometry.Brep[listA.Count];

            surFourPoints.ExpireSolution(true);
            doc.AddObject(surFourPoints, false);
            surFourPoints.Params.Output[0].CollectData();
            var sur4Pt = new Point3d[surFourPoints.Params.Output[0].VolatileDataCount];
            var curves = new Curve[surFourPoints.Params.Output[0].VolatileDataCount][];
            var joinCurves = new Curve[surFourPoints.Params.Output[0].VolatileDataCount];
            var extrusions = new Extrusion[surFourPoints.Params.Output[0].VolatileDataCount];

            var extrusionFaces = new Curve[extrusions.Length];

            for (int i = 0; i < surFourPoints.Params.Output[0].VolatileDataCount; ++i)
            {

                surfaceFourPoints.Add(surFourPoints.Params.Output[0].VolatileData.get_Branch(0)[i]);
                Grasshopper.Kernel.GH_Convert.ToGHSurface(surfaceFourPoints[i], GH_Conversion.Both, ref surfaces[i]);
                Grasshopper.Kernel.GH_Convert.ToBrep(surfaces[i], ref breps[i], GH_Conversion.Both);
                curves[i] = breps[i].GetWireframe(0);
                joinCurves[i] = Curve.JoinCurves(curves[i])[0];
                extrusions[i] = Extrusion.Create(joinCurves[i], GetRandomNumber(0.0, 0.5, joinCurves.Length)[i], true);
                extrusionFaces[i] = extrusions[i].GetWireframe()[1];

            }

            extrusionFacesOut = extrusionFaces.ToList();
            return extrusions;

        }

        private int[] MeshFaces(Mesh rhinoMesh, out int[] faceA, out int[] faceB, out int[] faceC)
        {

            faceA = new int[rhinoMesh.Faces.Count];
            faceB = new int[rhinoMesh.Faces.Count];
            faceC = new int[rhinoMesh.Faces.Count];
            var faceD = new int[rhinoMesh.Faces.Count];

            for (int i = 0; i < rhinoMesh.Faces.Count; ++i)
            {

                faceA[i] = rhinoMesh.Faces[i].A;
                faceB[i] = rhinoMesh.Faces[i].B;
                faceC[i] = rhinoMesh.Faces[i].C;
                faceD[i] = rhinoMesh.Faces[i].D;

            }

            return faceD;

        }

        private Point3d[] DeconstructMesh(GH_Mesh ghMesh, out List<object> faceMesh)
        {

            faceMesh = new List<object>();
            var verticesMesh = new List<object>();

            var deMesh = new SurfaceComponents.MeshComponents.Component_DeconstructMesh();
            var meshParams = deMesh.Params.Input[0] as Grasshopper.Kernel.GH_PersistentParam<Grasshopper.Kernel.Types.GH_Mesh>;
            meshParams.PersistentData.ClearData();
            meshParams.PersistentData.Append(ghMesh);

            var doc = new Grasshopper.Kernel.GH_Document();
            deMesh.ExpireSolution(true);
            doc.AddObject(deMesh, false);
            deMesh.Params.Output[0].CollectData();
            var rhinoVerMesh = new Point3d[deMesh.Params.Output[0].VolatileDataCount];
            for (int i = 0; i < deMesh.Params.Output[0].VolatileDataCount; ++i)
            {

                verticesMesh.Add(deMesh.Params.Output[0].VolatileData.get_Branch(0)[i]);
                Grasshopper.Kernel.GH_Convert.ToPoint3d(verticesMesh[i], ref rhinoVerMesh[i], GH_Conversion.Both);

            }

            for (int i = 0; i < deMesh.Params.Output[1].VolatileDataCount; ++i)
            {

                faceMesh.Add(deMesh.Params.Output[1].VolatileData.get_Branch(0)[i]);

            }

            return rhinoVerMesh;

        }

        private GH_Mesh GHMesh(Mesh rhinoMesh)
        {

            var ghMesh = new Grasshopper.Kernel.Types.GH_Mesh();

            Grasshopper.Kernel.GH_Convert.ToGHMesh(rhinoMesh, GH_Conversion.Both, ref ghMesh);

            return ghMesh;

        }

        private Mesh DelaunayPoints(List<Point3d> points)
        {

            var delaunayMesh = new TriangulationComponents.Component_Delaunay();

            var delPoints = delaunayMesh.Params.Input[0] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            delPoints.PersistentData.ClearData();
            for (int i = 0; i < points.Count; ++i)
            {

                delPoints.PersistentData.Append(new GH_Point(points[i]));

            }

            delaunayMesh.ExpireSolution(true);

            var doc = new Grasshopper.Kernel.GH_Document();
            doc.AddObject(delaunayMesh, false);

            delaunayMesh.Params.Output[0].CollectData();

            var delaunay = delaunayMesh.Params.Output[0].VolatileData.get_Branch(0)[0];

            var mesh = new Mesh();

            Grasshopper.Kernel.GH_Convert.ToMesh(delaunay, ref mesh, GH_Conversion.Both);

            return mesh;

        }

        private Mesh DelaunayMesh(List<Point3d> Points)
        {

            var nodeTwoList = new Grasshopper.Kernel.Geometry.Node2List(Points);
            var delaunayFaces = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Faces(nodeTwoList, 1);
            var delMesh = Grasshopper.Kernel.Geometry.Delaunay.Solver.Solve_Mesh(nodeTwoList, 1, ref delaunayFaces);
            delMesh.Weld(Math.PI);

            return delMesh;

        }

        public Mesh Subdivide(Mesh m, double threshold, bool sierpinsky)
        {
            List<MeshFace> newFaces = new List<MeshFace>();

            foreach (MeshFace mf in m.Faces)
            {
                double mfarea = AreaOfTriangle(m, mf);
                if (mfarea > threshold)
                {
                    m.Vertices.AddVertices(FaceMidPoints(m, mf));
                    newFaces.Add(new MeshFace(mf.A, m.Vertices.Count - 3, m.Vertices.Count - 1));
                    newFaces.Add(new MeshFace(m.Vertices.Count - 3, mf.B, m.Vertices.Count - 2));
                    newFaces.Add(new MeshFace(m.Vertices.Count - 1, m.Vertices.Count - 2, mf.C));
                    if (sierpinsky == false)
                        newFaces.Add(new MeshFace(m.Vertices.Count - 3, m.Vertices.Count - 2, m.Vertices.Count - 1));
                }
                else newFaces.Add(mf);
            }

            m.Faces.Clear();
            m.Faces.AddFaces(newFaces);
            newFaces.Clear();
            return m;

        }

        public List<Point3d> FaceMidPoints(Mesh m, MeshFace mf)
        {
            var rtnlist = new List<Point3d>();
            rtnlist.Add(MidPoint(m.Vertices[mf.A], m.Vertices[mf.B]));
            rtnlist.Add(MidPoint(m.Vertices[mf.B], m.Vertices[mf.C]));
            rtnlist.Add(MidPoint(m.Vertices[mf.C], m.Vertices[mf.A]));
            return rtnlist;
        }

        public Point3d MidPoint(Point3d pt1, Point3d pt2)
        {
            return new Point3d(0.5 * (pt1.X + pt2.X), 0.5 * (pt1.Y + pt2.Y), 0.5 * (pt1.Z + pt2.Z));
        }

        public static Mesh Triangulate(Mesh x)
        {
            int facecount = x.Faces.Count;
            for (int i = 0; i < facecount; i++)
            {
                var mf = x.Faces[i];
                if (mf.IsQuad)
                {
                    double dist1 = x.Vertices[mf.A].DistanceTo(x.Vertices[mf.C]);
                    double dist2 = x.Vertices[mf.B].DistanceTo(x.Vertices[mf.D]);
                    if (dist1 > dist2)
                    {
                        x.Faces.AddFace(mf.A, mf.B, mf.D);
                        x.Faces.AddFace(mf.B, mf.C, mf.D);
                    }
                    else
                    {
                        x.Faces.AddFace(mf.A, mf.B, mf.C);
                        x.Faces.AddFace(mf.A, mf.C, mf.D);
                    }
                }
            }

            var newfaces = new List<MeshFace>();
            foreach (var mf in x.Faces)
            {
                if (mf.IsTriangle) newfaces.Add(mf);
            }

            x.Faces.Clear();
            x.Faces.AddFaces(newfaces);
            return x;
        }

        public double AreaOfTriangle(Point3d pt1, Point3d pt2, Point3d pt3)
        {
            double a = pt1.DistanceTo(pt2);
            double b = pt2.DistanceTo(pt3);
            double c = pt3.DistanceTo(pt1);
            double s = (a + b + c) / 2;
            return Math.Sqrt(s * (s - a) * (s - b) * (s - c));
        }

        public double AreaOfTriangle(Mesh m, MeshFace mf)
        {
            return AreaOfTriangle(m.Vertices[mf.A], m.Vertices[mf.B], m.Vertices[mf.C]);
        }

        private Point3d ListItem(List<Point3d> points, int faceIndex)
        {

            if (faceIndex > points.Count) faceIndex = (faceIndex % points.Count);

            return points[faceIndex];

        }

        public double[] GetRandomNumber(double minimum, double maximum, int count)
        {
            Random random = new Random();

            var rand = new Double[count];

            for (int i = 0; i < count; ++i)
            {

                rand[i] = random.NextDouble() * (maximum - minimum) + minimum;

            }

            return rand;

        }

    }

}
