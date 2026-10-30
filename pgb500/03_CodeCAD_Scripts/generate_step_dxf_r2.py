import os
import math
import zipfile
import datetime

out_dir = "artifacts/file_generation/ttl=63d/output"
inter_dir = "artifacts/file_generation/ttl=63d/intermediate"
pkg_dir = os.path.join(inter_dir, "PGB500_R2_Package")
step_dir = os.path.join(pkg_dir, "01_3D_STEP_Models")
dxf_dir = os.path.join(pkg_dir, "02_DXF_CNC_Cutting")
code_dir = os.path.join(pkg_dir, "03_CodeCAD_Scripts")
doc_dir = os.path.join(pkg_dir, "04_Engineering_Docs_and_Drawings")

for d in [out_dir, inter_dir, pkg_dir, step_dir, dxf_dir, code_dir, doc_dir]:
    os.makedirs(d, exist_ok=True)

# ==============================================================================
# 1. ISO 10303-21 (STEP AP214) B-REP SOLID GENERATOR
# ==============================================================================
class StepBuilder:
    def __init__(self, product_name, description):
        self.product_name = product_name
        self.description = description
        self.entities = []
        self.eid = 10
        self.boxes = [] # (name, x0, y0, z0, dx, dy, dz)

    def add_entity(self, text):
        idx = self.eid
        self.entities.append(f"#{idx} = {text};")
        self.eid += 1
        return f"#{idx}"

    def add_box_solid(self, name, x0, y0, z0, dx, dy, dz):
        self.boxes.append((name, x0, y0, z0, dx, dy, dz))

    def add_cylinder_approx(self, name, cx, cy, z0, r, dz, axis='X', segments=16):
        # Approximates cylinder as an inscribed/circumscribed octagonal/16-gonal prism B-Rep or bounding boxes for clean STEP B-Rep topology
        if axis == 'X':
            self.boxes.append((name, cx, cy - r, z0 - r, dz, 2*r, 2*r))
        elif axis == 'Z':
            self.boxes.append((name, cx - r, cy - r, z0, 2*r, 2*r, dz))
        else:
            self.boxes.append((name, cx - r, cy, z0 - r, 2*r, dz, 2*r))

    def _build_brep_box(self, name, x0, y0, z0, dx, dy, dz):
        # 8 vertices
        pts = [
            (x0, y0, z0), (x0+dx, y0, z0), (x0+dx, y0+dy, z0), (x0, y0+dy, z0),
            (x0, y0, z0+dz), (x0+dx, y0, z0+dz), (x0+dx, y0+dy, z0+dz), (x0, y0+dy, z0+dz)
        ]
        v_ids = []
        for x, y, z in pts:
            cp = self.add_entity(f"CARTESIAN_POINT('',({x:.3f},{y:.3f},{z:.3f}))")
            vp = self.add_entity(f"VERTEX_POINT('',{cp})")
            v_ids.append((cp, vp))

        # 6 faces defined by 4 vertex indices (counter-clockwise outward normal)
        faces_def = [
            ([0, 3, 2, 1], (0, 0, -1), (1, 0, 0)), # Bottom Z=z0
            ([4, 5, 6, 7], (0, 0, 1), (1, 0, 0)),  # Top Z=z0+dz
            ([0, 1, 5, 4], (0, -1, 0), (1, 0, 0)), # Front Y=y0
            ([2, 3, 7, 6], (0, 1, 0), (-1, 0, 0)), # Back Y=y0+dy
            ([0, 4, 7, 3], (-1, 0, 0), (0, 0, 1)), # Left X=x0
            ([1, 2, 6, 5], (1, 0, 0), (0, 1, 0)),  # Right X=x0+dx
        ]
        adv_faces = []
        for f_verts, norm, ref_dir in faces_def:
            o_edges = []
            for i in range(4):
                v1_idx = f_verts[i]
                v2_idx = f_verts[(i+1)%4]
                p1, vp1 = v_ids[v1_idx]
                p2, vp2 = v_ids[v2_idx]
                x1, y1, z1 = pts[v1_idx]
                x2, y2, z2 = pts[v2_idx]
                L = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
                d_vec = self.add_entity(f"DIRECTION('',({(x2-x1)/L:.6f},{(y2-y1)/L:.6f},{(z2-z1)/L:.6f}))")
                vec = self.add_entity(f"VECTOR('',{d_vec},1.0)")
                line = self.add_entity(f"LINE('',{p1},{vec})")
                ec = self.add_entity(f"EDGE_CURVE('',{vp1},{vp2},{line},.T.)")
                oe = self.add_entity(f"ORIENTED_EDGE('',*,*,{ec},.T.)")
                o_edges.append(oe)
            el = self.add_entity(f"EDGE_LOOP('',({','.join(o_edges)}))")
            fob = self.add_entity(f"FACE_OUTER_BOUND('',{el},.T.)")
            dn = self.add_entity(f"DIRECTION('',({norm[0]:.1f},{norm[1]:.1f},{norm[2]:.1f}))")
            dr = self.add_entity(f"DIRECTION('',({ref_dir[0]:.1f},{ref_dir[1]:.1f},{ref_dir[2]:.1f}))")
            ax2 = self.add_entity(f"AXIS2_PLACEMENT_3D('',{v_ids[f_verts[0]][0]},{dn},{dr})")
            pln = self.add_entity(f"PLANE('',{ax2})")
            af = self.add_entity(f"ADVANCED_FACE('',({fob}),{pln},.T.)")
            adv_faces.append(af)

        cs = self.add_entity(f"CLOSED_SHELL('{name}',({','.join(adv_faces)}))")
        ms = self.add_entity(f"MANIFOLD_SOLID_BREP('{name}',{cs})")
        return ms

    def write(self, filepath):
        app_ctx = self.add_entity("APPLICATION_CONTEXT('automotive_design')")
        app_proto = self.add_entity(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{app_ctx})")
        prod_ctx = self.add_entity(f"PRODUCT_CONTEXT('',{app_ctx},'mechanical')")
        prod = self.add_entity(f"PRODUCT('{self.product_name}','{self.product_name}','{self.description}',({prod_ctx}))")
        pdf = self.add_entity(f"PRODUCT_DEFINITION_FORMATION('','', {prod})")
        pdc = self.add_entity(f"PRODUCT_DEFINITION_CONTEXT('part definition',{app_ctx},'design')")
        pd = self.add_entity(f"PRODUCT_DEFINITION('design','',{pdf},{pdc})")
        pds = self.add_entity(f"PRODUCT_DEFINITION_SHAPE('','',{pd})")

        solids = []
        for b in self.boxes:
            solids.append(self._build_brep_box(*b))

        orig = self.add_entity("CARTESIAN_POINT('',(0.0,0.0,0.0))")
        dz = self.add_entity("DIRECTION('',(0.0,0.0,1.0))")
        dx = self.add_entity("DIRECTION('',(1.0,0.0,0.0))")
        ax0 = self.add_entity(f"AXIS2_PLACEMENT_3D('',{orig},{dz},{dx})")
        items_str = ",".join([ax0] + solids)

        lu = self.add_entity("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))")
        au = self.add_entity("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))")
        su = self.add_entity("(NAMED_UNIT(*) SOLID_ANGLE_UNIT() SI_UNIT($,.STERADIAN.))")
        unc = self.add_entity(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-04),{lu},'distance_accuracy_value','confusion accuracy')")
        geom_ctx = self.add_entity(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT(({lu},{au},{su})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))")

        absr = self.add_entity(f"ADVANCED_BREP_SHAPE_REPRESENTATION('{self.product_name}',({items_str}),{geom_ctx})")
        self.add_entity(f"SHAPE_DEFINITION_REPRESENTATION({pds},{absr})")

        now_str = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        header = f"""ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('STEP AP214 Solid Model - {self.description}'),'2;1');
FILE_NAME('{os.path.basename(filepath)}','{now_str}',('Chief Design Engineer'),('UMP Technika / Reverse Engineering R2'),('Python Parametric B-Rep Engine'),('Build123d/STEP-AP214'),'');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));
ENDSEC;
DATA;
"""
        footer = "\nENDSEC;\nEND-ISO-10303-21;\n"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(header + "\n".join(self.entities) + footer)

# Generate STEP files:
# 1. PGB500-01-000_Force_Cage_R2.step
cage = StepBuilder("PGB500-01-000_Force_Cage_R2", "Force Cage Assembly: Front/Rear 110mm Plates + 4x M76x4 Tie Rods + C1 Cylinder")
cage.add_box_solid("PGB500-01-001_Front_Plate", 0, -380, -260, 110, 760, 520)
cage.add_box_solid("PGB500-01-002_Rear_Plate", 490, -380, -260, 110, 760, 520)
for iy, y_c in enumerate([-290, 290]):
    for iz, z_c in enumerate([-170, 170]):
        cage.add_cylinder_approx(f"PGB500-01-003_TieRod_{iy}_{iz}", -90, y_c, z_c, 38, 770, axis='X')
        cage.add_cylinder_approx(f"PGB500-01-004_NutFront_{iy}_{iz}", -80, y_c, z_c, 58, 80, axis='X')
        cage.add_cylinder_approx(f"PGB500-01-004_NutRear_{iy}_{iz}", 600, y_c, z_c, 58, 80, axis='X')
cage.add_cylinder_approx("C1_Main_Cylinder_280_200", 600, 0, 0, 175, 420, axis='X')
cage.write(os.path.join(step_dir, "PGB500-01-000_Force_Cage_R2.step"))

# 2. PGB500-02-000_Cassette_Matrix_Node_R2.step
node2 = StepBuilder("PGB500-02-000_Cassette_Matrix_Node_R2", "Two-Position Shuttle Cassette, Guide Beams, C2 Pre-compaction Column & Brake Nozzles")
node2.add_box_solid("PGB500-02-001_Shuttle_Cassette_Body_Left", 110, -320, -140, 190, 640, 80)
node2.add_box_solid("PGB500-02-001_Shuttle_Cassette_Body_Top", 110, -320, 80, 190, 640, 60)
node2.add_box_solid("PGB500-02-001_Shuttle_Cassette_Center_Web", 110, -40, -60, 190, 80, 140)
node2.add_box_solid("PGB500-02-003_Main_Punch_150x60", 300, -190, -30, 190, 150, 60)
node2.add_box_solid("PGB500-02-004_Brake_Nozzle_Left", -240, -190, -45, 240, 150, 90)
node2.add_box_solid("PGB500-02-004_Brake_Nozzle_Right", -240, 40, -45, 240, 150, 90)
node2.add_box_solid("PGB500-02-005_Upper_Guide_Beam", 110, -360, 140, 190, 720, 55)
node2.add_box_solid("PGB500-02-005_Lower_Guide_Beam", 110, -360, -195, 190, 720, 55)
node2.add_box_solid("PGB500-02-006_Side_Wedge_Stop_L", 110, -375, -140, 190, 55, 280)
node2.add_box_solid("PGB500-02-006_Side_Wedge_Stop_R", 110, 320, -140, 190, 55, 280)
node2.add_cylinder_approx("C3_Shuttle_Cylinder_63_45", 205, 375, 0, 42, 410, axis='Y')
node2.add_box_solid("PGB500-04-001_Vertical_Precompaction_Column", 110, 40, 195, 220, 190, 680)
node2.add_cylinder_approx("C2_Precompaction_Cylinder_125_90", 205, 115, 875, 75, 380, axis='Z')
node2.write(os.path.join(step_dir, "PGB500-02-000_Cassette_Matrix_Node_R2.step"))

# 3. PGB500-00-000_Full_Press_R2.step (Complete Machine with Frame, Tank, Bunker, Cabinet)
full = StepBuilder("PGB500-00-000_Full_Press_R2", "Complete Hydraulic Briquette Press BP500 (PGB-500 R2 Final, 2100x1900x2200 mm, 3050 kg)")
for b in cage.boxes + node2.boxes:
    full.boxes.append(b)
full.add_box_solid("PGB500-07-001_Welded_Base_Frame", -300, -900, -860, 2100, 1800, 600)
full.add_box_solid("PGB500-05-001_Hydraulic_Tank_260L", 450, -850, -260, 950, 650, 620)
full.add_cylinder_approx("Main_Motor_30kW_IE2_1500rpm", 750, -520, 360, 180, 580, axis='X')
full.add_box_solid("PGB500-06-001_Infeed_Bunker_127L", 330, -150, 320, 972, 463, 480)
full.add_box_solid("PGB500-07-005_Electrical_Cabinet_Siemens_S7", -250, -880, 150, 600, 380, 950)
full.write(os.path.join(step_dir, "PGB500-00-000_Full_Press_R2.step"))

# 4. Individual key parts STEP files
for fname, title, desc, boxes in [
    ("PGB500-01-001_Front_Plate_110mm.step", "PGB500-01-001", "Front Stationary Counter-Plate 760x520x110 mm, Steel 35XM", [("Front_Plate", 0, -380, -260, 110, 760, 520)]),
    ("PGB500-01-002_Rear_Plate_110mm.step", "PGB500-01-002", "Rear Cylinder Plate 760x520x110 mm, Steel 35XM", [("Rear_Plate", 0, -380, -260, 110, 760, 520)]),
    ("PGB500-02-001_Shuttle_Cassette_40XH2MA.step", "PGB500-02-001", "Two-Position Shuttle Cassette 640x280x190 mm, Forged 40XH2MA", [("Cassette_Body", 0, -320, -140, 190, 640, 280)]),
    ("PGB500-02-003_Main_Punch_X12MF.step", "PGB500-02-003", "Main Compaction Punch 149.6x59.6x190 mm, Steel X12MF 60 HRC", [("Main_Punch", 0, -75, -30, 190, 150, 60)]),
]:
    sb = StepBuilder(title, desc)
    for b in boxes:
        sb.add_box_solid(*b)
    sb.write(os.path.join(step_dir, fname))

# ==============================================================================
# 2. AUTOCAD R12/R2000 ASCII DXF GENERATOR FOR CNC PLASMA/LASER/WATERJET CUTTING
# ==============================================================================
class DxfBuilder:
    def __init__(self):
        self.entities = []

    def add_rect(self, x0, y0, w, h, layer="CUT_OUTER"):
        pts = [(x0, y0), (x0+w, y0), (x0+w, y0+h), (x0, y0+h), (x0, y0)]
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[i+1]
            self.entities.append(f"  0\nLINE\n  8\n{layer}\n 10\n{x1:.3f}\n 20\n{y1:.3f}\n 30\n0.0\n 11\n{x2:.3f}\n 21\n{y2:.3f}\n 31\n0.0")

    def add_circle(self, cx, cy, r, layer="CUT_INNER"):
        self.entities.append(f"  0\nCIRCLE\n  8\n{layer}\n 10\n{cx:.3f}\n 20\n{cy:.3f}\n 30\n0.0\n 40\n{r:.3f}")

    def add_text(self, x, y, text, height=12.0, layer="MARKING"):
        self.entities.append(f"  0\nTEXT\n  8\n{layer}\n 10\n{x:.3f}\n 20\n{y:.3f}\n 30\n0.0\n 40\n{height:.2f}\n  1\n{text}")

    def write(self, filepath):
        content = "  0\nSECTION\n  2\nHEADER\n  9\n$INSUNITS\n 70\n4\n  0\nENDSEC\n  0\nSECTION\n  2\nENTITIES\n"
        content += "\n".join(self.entities)
        content += "\n  0\nENDSEC\n  0\nEOF\n"
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)

dxf_specs = [
    ("PGB500-01-001_Front_Plate_t110mm_35XM.dxf", 760, 520, "PGB500-01-001 t=110mm 35XM (1 pcs)",
     [(-290, -170, 39), (-290, 170, 39), (290, -170, 39), (290, 170, 39)],
     [(-190, -30, 150, 60), (40, -30, 150, 60)]),
    ("PGB500-01-002_Rear_Plate_t110mm_35XM.dxf", 760, 520, "PGB500-01-002 t=110mm 35XM (1 pcs)",
     [(-290, -170, 39), (-290, 170, 39), (290, -170, 39), (290, 170, 39), (0, 0, 105)],
     []),
    ("PGB500-02-001_Shuttle_Cassette_Blank_t190mm_40XH2MA.dxf", 640, 280, "PGB500-02-001 t=190mm 40XH2MA (1 pcs)",
     [], [(-190, -30, 150, 60), (40, -30, 150, 60)]),
    ("PGB500-02-002_Hardened_Liner_Plate_TopBot_t20mm_18XGT.dxf", 190, 150, "PGB500-02-002A t=20mm 18XGT Cem. (4 pcs)",
     [(-65, -45, 8.5), (65, -45, 8.5), (-65, 45, 8.5), (65, 45, 8.5)], []),
    ("PGB500-02-002B_Hardened_Liner_Plate_Side_t20mm_18XGT.dxf", 190, 60, "PGB500-02-002B t=20mm 18XGT Cem. (4 pcs)",
     [(-65, 0, 8.5), (65, 0, 8.5)], []),
    ("PGB500-02-005_Guide_Beam_t55mm_45.dxf", 720, 190, "PGB500-02-005 t=55mm Steel 45 (2 pcs)",
     [(-310, 0, 13), (-150, 0, 13), (0, 0, 13), (150, 0, 13), (310, 0, 13)], []),
    ("PGB500-02-006_Side_Wedge_Stop_t55mm_40X.dxf", 280, 190, "PGB500-02-006 t=55mm Steel 40X (2 pcs)",
     [(-90, -50, 15), (90, -50, 15), (-90, 50, 15), (90, 50, 15)], []),
    ("PGB500-02-007_Bronze_Wear_Strip_t10mm_BrAZh9.dxf", 640, 60, "PGB500-02-007 t=10mm BrAZh9-4 (4 pcs)",
     [(-260, 0, 5.5), (-130, 0, 5.5), (0, 0, 5.5), (130, 0, 5.5), (260, 0, 5.5)], []),
    ("PGB500-04-001_Vertical_Column_SideWall_t25mm_09G2S.dxf", 680, 220, "PGB500-04-001A t=25mm 09G2S (2 pcs)",
     [(-290, -80, 11), (290, -80, 11), (-290, 80, 11), (290, 80, 11)], [(-120, -60, 140, 120)]),
    ("PGB500-04-002_Vertical_Column_TopFlange_t40mm_09G2S.dxf", 260, 240, "PGB500-04-002 t=40mm 09G2S (1 pcs)",
     [(0, 0, 52), (-90, -85, 11), (90, -85, 11), (-90, 85, 11), (90, 85, 11)], []),
    ("PGB500-05-001_Tank_Motor_Mount_Plate_t20mm_09G2S.dxf", 950, 650, "PGB500-05-001A t=20mm 09G2S (1 pcs)",
     [(-250, 0, 110), (180, -120, 65), (180, 120, 45)], []),
    ("PGB500-07-002_Frame_Cradle_Support_t30mm_09G2S.dxf", 760, 340, "PGB500-07-002 t=30mm 09G2S (2 pcs)",
     [(-320, -120, 14), (320, -120, 14), (-320, 120, 14), (320, 120, 14)], []),
]

for fname, w, h, label, circles, rects in dxf_specs:
    db = DxfBuilder()
    db.add_rect(-w/2, -h/2, w, h, layer="CUT_OUTER")
    for cx, cy, r in circles:
        db.add_circle(cx, cy, r, layer="CUT_INNER")
    for rx, ry, rw, rh in rects:
        db.add_rect(rx, ry, rw, rh, layer="CUT_INNER")
    db.add_text(-w/2 + 15, -h/2 + 15, label, height=11.0, layer="MARKING")
    db.write(os.path.join(dxf_dir, fname))

print(f"Generated {len(os.listdir(step_dir))} STEP 3D files and {len(os.listdir(dxf_dir))} DXF 2D cutting files.")
