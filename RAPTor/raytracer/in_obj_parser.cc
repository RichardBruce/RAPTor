// #include "objloader.h"    // include first
// #include "common/math/point3.h"
// #include "common/misc/progressmonitor.h"
// #include "common/misc/stringutils.h"
// #include "triangulator.h"

// #include <cctype>
// #include <IL/il.h>

// //#define VERBOSE

// #ifdef VERBOSE
// #include <iostream>
// #endif    // VERBOSE

// using namespace sheep;
// using namespace std;

//namespace raptor_raytracer
//{
// OBJLoader::OBJLoader() {
//     m_keyword_table.Insert("d", D);
//     m_keyword_table.Insert("f", F);
//     m_keyword_table.Insert("g", G);
//     m_keyword_table.Insert("illum", ILLUM);
//     m_keyword_table.Insert("Ka", KA);
//     m_keyword_table.Insert("Kd", KD);
//     m_keyword_table.Insert("Ks", KS);
//     m_keyword_table.Insert("map_Kd", MAP_KD);
//     m_keyword_table.Insert("mtllib", MTLLIB);
//     m_keyword_table.Insert("newmtl", NEWMTL);
//     m_keyword_table.Insert("s", S);
//     m_keyword_table.Insert("Tr", TR);
//     m_keyword_table.Insert("usemtl", USEMTL);
//     m_keyword_table.Insert("v", V);
//     m_keyword_table.Insert("vn", VN);
//     m_keyword_table.Insert("vt", VT);
// }

// void OBJLoader::Load(const string &filename,
//                      IMeshBuilder &builder,
//                      int option_mask /*= DEFAULT_CONFIGURATION_BIT*/,
//                      ProgressMonitor *progmon /*= 0*/)
// {
//     m_path = StringUtils::GetPath(filename);

//     m_geombuilder = builder.GeometryBuilder();
//     m_matbuilder = builder.MaterialBuilder();

//     m_option_mask = option_mask;

//     m_progmon = progmon;

//     m_file = fopen(filename.c_str(), "r");

//     if(!m_file)
//         throw FileNotFoundException(filename);

//     if(m_progmon) {
//         fseek(m_file, 0, SEEK_END);
//         m_progmon->StartJob(0, ftell(m_file));
//         fseek(m_file, 0, SEEK_SET);
//     }

//     m_line = 1;

//     m_vertices.clear();
//     m_normals.clear();
//     m_texcoords.clear();

//     m_create_submesh = true;
//     m_set_submesh_material = false;
//     m_submesh_material_id = 0;
//     m_inside_submesh_def = false;

//     parse_file();

//     if(m_inside_submesh_def)
//         m_geombuilder->EndSubMesh();

//     if(m_progmon)
//         m_progmon->Done();

//     fclose(m_file);
// }

// void OBJLoader::parse_error() {
//     //!\todo Close files and free resources.
//     throw ParsingException(m_line);
// }

// void OBJLoader::eat_leading_blanks() {
//     assert(m_file);

//     bool inside_comment = false;

//     while(true) {
//         int c = fgetc(m_file);

//         if(c == EOF)
//             break;

//         if(c == '\n')
//             ++m_line;

//         if(inside_comment) {
//             if(c == '\n')
//                 inside_comment = false;
//         } else {
//             if(c == '#')
//                 inside_comment = true;
//             else if(!isspace(c)) {
//                 assert(c != EOF);

//                 if(c == '\n')
//                     --m_line;

//                 ungetc(c, m_file);

//                 break;
//             }
//         }
//     }
// }

// void OBJLoader::eat_blanks() {
//     assert(m_file);

//     while(true) {
//         const int c = fgetc(m_file);

//         if(c == EOF)
//             break;

//         if(c == '\n')
//             ++m_line;

//         if(c == '\n' || !isspace(c)) {
//             assert(c != EOF);

//             if(c == '\n')
//                 --m_line;

//             ungetc(c, m_file);

//             break;
//         }
//     }
// }

// void OBJLoader::eat_line() {
//     assert(m_file);

//     while(true) {
//         const int c = fgetc(m_file);

//         if(c == EOF)
//             break;

//         if(c == '\n') {
//             ++m_line;
//             break;
//         }
//     }
// }

// int OBJLoader::accept_integer() {
//     assert(m_file);

//     const int c = look_ahead();

//     if(c == EOF || isspace(c))
//         parse_error();

//     int n;

//     if(fscanf(m_file, "%9d", &n) < 1)
//         parse_error();

//     return n;
// }

// double OBJLoader::accept_double() {
//     assert(m_file);

//     const int c = look_ahead();

//     if(c == EOF || isspace(c))
//         parse_error();

//     double d;

//     if(fscanf(m_file, "%9lf", &d) < 1)
//         parse_error();

//     return d;
// }

// string OBJLoader::accept_string() {
//     assert(m_file);

//     int c = fgetc(m_file);

//     if(c == EOF || isspace(c))
//         parse_error();

//     string s = "";

//     while(true) {
//         s += c;

//         c = fgetc(m_file);

//         if(c == EOF)
//             break;

//         if(c == '\n')
//             ++m_line;

//         if(isspace(c)) {
//             assert(c != EOF);

//             if(c == '\n')
//                 --m_line;

//             ungetc(c, m_file);

//             break;
//         }
//     }

//     return s;
// }

// void OBJLoader::select_submesh() {
//     if(m_create_submesh) {
//         // Close the current submesh.
//         if(m_inside_submesh_def)
//             m_geombuilder->EndSubMesh();

//         // Begin a new submesh.
// #ifdef VERBOSE
//         cerr << "[OBJLoader] Loading mesh..." << endl;
// #endif    // VERBOSE
//         m_geombuilder->BeginSubMesh("");
//         m_inside_submesh_def = true;

//         // Set submesh material if a material has been specified via the 'usemtl' statement.
//         if(m_set_submesh_material) {
//             m_geombuilder->SetMaterial(m_submesh_material_id);
//             m_set_submesh_material = false;
//             m_submesh_material_id = 0;
//         }

//         m_global_vertex_id.clear();

//         m_create_submesh = false;
//     }
// }

// void OBJLoader::parse_file() {
//     int parsed_statements = 0;

//     while(true) {
//         eat_leading_blanks();

//         if(look_ahead() == EOF)
//             break;

//         const string keyword = accept_string();

//         switch(m_keyword_table.GetSymbol(keyword)) {
//         case F:
//             select_submesh();
//             parse_f_statement();
//             break;
//         case MTLLIB:
//             parse_mtllib_statement();
//             break;
//         case USEMTL:
//             parse_usemtl_statement();
//             break;
//         case V:
//             parse_v_statement();
//             break;
//         case VN:
//             parse_vn_statement();
//             break;
//         case VT:
//             parse_vt_statement();
//             break;
//         default:
//             eat_line();    // ignore unknown statements
//         }

//         if(m_progmon) {
//             //!\todo Fine tune the number of statements to parse between progress updates.
//             if((parsed_statements & 127) == 0)
//                 m_progmon->SetJobProgress(ftell(m_file));
//         }

//         ++parsed_statements;
//     }
// }

// namespace {
//     template<typename T>
//     int fix_vector_index(const vector<T> &v, int index) {
//         if(index > 0) {
//             assert(index >= 1 && index <= v.size());
//             return index - 1;
//         } else {
//             assert(-index >= 1 && -index <= v.size());
//             return v.size() + index;
//         }
//     }
// }

// void OBJLoader::parse_f_statement() {
//     vector<IMeshBuilder::FeatureId> vertex_id, texcoord_id, normal_id;

//     vertex_id.reserve(3);
//     texcoord_id.reserve(3);
//     normal_id.reserve(3);

//     vector<Point3> polygon_vertices;
//     polygon_vertices.reserve(3);

//     while(true) {
//         eat_blanks();

//         if(is_eol())
//             break;

//         bool has_texcoord = false;
//         bool has_normal = false;
//         int v, vt, vn;
//         int state = 0;
//         bool terminate = false;
//         int c;

//         while(!terminate) {
//             switch(state) {
//             // Recognized (epsilon)
//             // Accept n(/((n(/n)?)|(/n)))?
//             case 0:
//                 v = accept_integer();
//                 state = 1;
//                 break;
//             // Recognized n
//             // Accept (/((n(/n)?)|(/n)))?
//             case 1:
//                 c = look_ahead();
//                 if(isspace(c))
//                     goto terminate;
//                 else if(c == '/') {
//                     accept_char('/');
//                     state = 2;
//                 } else parse_error();
//                 break;
//             // Recognized n/
//             // Accept (n(/n)?)|(/n)
//             case 2:
//                 c = look_ahead();
//                 if(c == '/') {
//                     accept_char('/');
//                     state = 4;
//                 } else {
//                     vt = accept_integer();
//                     has_texcoord = true;
//                     state = 3;
//                 }
//                 break;
//             // Recognized n/n
//             // Accept (/n)?
//             case 3:
//                 c = look_ahead();
//                 if(isspace(c))
//                     goto terminate;
//                 else if(c == '/') {
//                     accept_char('/');
//                     state = 4;
//                 } else parse_error();
//                 break;
//             // Recognized n//
//             // Accept n
//             case 4:
//                 vn = accept_integer();
//                 has_normal = true;
//                 goto terminate;
//                 break;
//             default:
//                 assert(!"Invalid state.");
//             }
//         }

// terminate:

//         v = fix_vector_index(m_vertices, v);

//         if(m_global_vertex_id.find(v) == m_global_vertex_id.end()) {
//             const IMeshBuilder::FeatureId id = m_geombuilder->AppendVertex(m_vertices[v]);
//             m_global_vertex_id[v] = id;
//             vertex_id.push_back(id);
//         } else vertex_id.push_back(m_global_vertex_id[v]);

//         polygon_vertices.push_back(m_vertices[v]);

//         if(has_texcoord) {
//             vt = fix_vector_index(m_texcoords, vt);
//             const IMeshBuilder::FeatureId id = m_geombuilder->AppendTexCoord(m_texcoords[vt]);
//             texcoord_id.push_back(id);
//         }

//         if(has_normal) {
//             vn = fix_vector_index(m_normals, vn);
//             const IMeshBuilder::FeatureId id = m_geombuilder->AppendNormal(m_normals[vn]);
//             normal_id.push_back(id);
//         }
//     }

//     accept_newline();

//     const int n = vertex_id.size();

//     if(n < 3) {
//         if(m_option_mask & STOP_ON_INVALID_FACE) {
//             m_geombuilder->EndSubMesh();
//             throw InvalidFaceException(m_line);
//         } else return;    // ignore invalid faces
//     }

//     assert(texcoord_id.size() == 0 || texcoord_id.size() == n);
//     assert(normal_id.size() == 0 || normal_id.size() == n);

//     const bool has_texcoord = texcoord_id.size() == n;
//     const bool has_normal = normal_id.size() == n;

//     vector<int> triangles;

//     if(polygon_vertices.size() > 3 && !(m_option_mask & DISABLE_TRIANGULATION_BIT)) {
//         if(!TriangulatePolygon3(polygon_vertices, &triangles)) {
//             // Could not triangulate this polygon, because it is either
//             // self-intersecting or not planar.
//             if(m_option_mask & STOP_ON_TRIANGULATION_ERROR_BIT) {
//                 m_geombuilder->EndSubMesh();
//                 throw TriangulationException(m_line);
//             } else {
// #ifdef VERBOSE
//                 cerr << "[OBJLoader] Warning: could not triangulate the following polygon:" << endl;
//                 for(vector<Point3>::const_iterator i = polygon_vertices.begin(), e = polygon_vertices.end(); i != e; ++i) {
//                     cerr << "(" << i->m_x << ", " << i->m_y << ", " << i->m_z << ") ";
//                 }
//                 cerr << endl;
// #endif    // VERBOSE
//                 return;    // ignore problematic polygons
//             }
//         }
//     } else {
//         // Identity mapping.
//         for(int i = 0; i < polygon_vertices.size(); ++i)
//             triangles.push_back(i);
//     }

//     for(int i = 0; i < triangles.size(); i += 3) {
//         IMeshBuilder::FeatureId v[3];

//         v[0] = vertex_id[triangles[i]];
//         v[1] = vertex_id[triangles[i + 1]];
//         v[2] = vertex_id[triangles[i + 2]];

//         IMeshBuilder::FeatureId face_id = m_geombuilder->AppendFace(3, v);

//         if(has_texcoord) {
//             IMeshBuilder::FeatureId vt[3];

//             vt[0] = texcoord_id[triangles[i]];
//             vt[1] = texcoord_id[triangles[i + 1]];
//             vt[2] = texcoord_id[triangles[i + 2]];

//             m_geombuilder->SetFaceTexCoords(face_id, 3, vt);
//         }

//         if(has_normal) {
//             IMeshBuilder::FeatureId vn[3];

//             vn[0] = normal_id[triangles[i]];
//             vn[1] = normal_id[triangles[i + 1]];
//             vn[2] = normal_id[triangles[i + 2]];

//             m_geombuilder->SetFaceNormals(face_id, 3, vn);
//         }
//     }
// }

// void OBJLoader::parse_mtllib_statement() {
//     eat_blanks();

//     string filename = accept_string();

//     if(m_matbuilder)
//         load_material_file(filename);

//     while(true) {
//         eat_blanks();

//         if(is_eol())
//             break;

//         filename = accept_string();

//         if(m_matbuilder)
//             load_material_file(filename);
//     }

//     accept_newline();
// }

// void OBJLoader::parse_usemtl_statement() {
//     eat_blanks();

//     if(is_eol()) {
//         // No material has been specified: new faces will be added
//         // to a new submesh, but no material will be applied to this
//         // new submesh.
//         m_create_submesh = true;
//         m_set_submesh_material = false;
//         m_submesh_material_id = 0;
//     } else {
//         const string material_name = accept_string();
//         string_to_feature_id_map::const_iterator i = m_material_id.find(material_name);

//         if(i != m_material_id.end()) {
//             // The requested material is defined: new faces will be added
//             // to a new submesh, and the specified material will be applied
//             // to this new submesh.
//             m_create_submesh = true;
//             m_set_submesh_material = true;
//             m_submesh_material_id = i->second;
//         } else {
//             // The requested material is not defined: new faces will be
//             // added to a new submesh, but no material will be applied to
//             // this new submesh.
//             m_create_submesh = true;
//             m_set_submesh_material = false;
//             m_submesh_material_id = 0;
//         }
//     }

//     eat_blanks();
//     accept_newline();
// }

// void OBJLoader::parse_v_statement() {
//     Vector3 v;

//     eat_blanks();
//     v.m_x = accept_double();

//     eat_blanks();
//     v.m_y = accept_double();

//     eat_blanks();
//     v.m_z = accept_double();

//     eat_blanks();
//     if(!is_eol())
//         accept_double();

//     eat_blanks();
//     accept_newline();

//     m_vertices.push_back(v);
// }

// void OBJLoader::parse_vt_statement() {
//     Vector2 v;

//     eat_blanks();
//     v.m_x = accept_double();

//     eat_blanks();
//     v.m_y = -accept_double();

//     eat_blanks();
//     if(!is_eol())
//         accept_double();

//     eat_blanks();
//     accept_newline();

//     m_texcoords.push_back(v);
// }

// void OBJLoader::parse_vn_statement() {
//     Vector3 n;

//     eat_blanks();
//     n.m_x = accept_double();

//     eat_blanks();
//     n.m_y = accept_double();

//     eat_blanks();
//     n.m_z = accept_double();

//     eat_blanks();
//     accept_newline();

//     n.Normalize();

//     m_normals.push_back(n);
// }

// void OBJLoader::load_material_file(const string &filename) {
//     //!\todo This is quite dirty, I must admit.
//     FILE * const file_copy = m_file;
//     const int line_copy = m_line;

//     m_file = TryOpeningFile(m_path, filename, "r");

//     if(m_file) {
//         m_line = 1;

//         m_inside_material_def = false;

//         parse_material_file();

//         if(m_inside_material_def) {
//             assert(m_matbuilder);
//             m_matbuilder->EndMaterial();
//         }
//     } else {
//         if(m_option_mask & STOP_ON_MISSING_FILE_BIT) {
//             //!\todo Close files and free resources.
//             throw FileNotFoundException(filename);
//         }

//         // Ignore missing material file.
//     }

//     m_file = file_copy;
//     m_line = line_copy;
// }

// void OBJLoader::parse_material_file() {
//     while(true) {
//         eat_leading_blanks();

//         if(look_ahead() == EOF)
//             break;

//         const string keyword = accept_string();

//         switch(m_keyword_table.GetSymbol(keyword)) {
//         case KA:
//             parse_ka_statement();
//             break;
//         case KD:
//             parse_kd_statement();
//             break;
//         case KS:
//             parse_ks_statement();
//             break;
//         case MAP_KD:
//             parse_map_kd_statement();
//             break;
//         case NEWMTL:
//             parse_newmtl_statement();
//             break;
//         default:
//             eat_line();    // ignore unknown statements
//         }
//     }
// }

// void OBJLoader::parse_ka_statement() {
//     eat_blanks();
//     const double r = accept_double();

//     eat_blanks();
//     const double g = accept_double();

//     eat_blanks();
//     const double b = accept_double();

//     eat_blanks();
//     accept_newline();

//     if(!m_inside_material_def)
//         parse_error();

//     assert(m_matbuilder);
//     m_matbuilder->SetAmbientColor(r, g, b);
// }

// void OBJLoader::parse_kd_statement() {
//     eat_blanks();
//     const double r = accept_double();

//     eat_blanks();
//     const double g = accept_double();

//     eat_blanks();
//     const double b = accept_double();

//     eat_blanks();
//     accept_newline();

//     if(!m_inside_material_def)
//         parse_error();

//     assert(m_matbuilder);
//     m_matbuilder->SetDiffuseColor(r, g, b);
// }

// void OBJLoader::parse_ks_statement() {
//     eat_blanks();
//     const double r = accept_double();

//     eat_blanks();
//     const double g = accept_double();

//     eat_blanks();
//     const double b = accept_double();

//     eat_blanks();
//     accept_newline();

//     if(!m_inside_material_def)
//         parse_error();

//     assert(m_matbuilder);
//     m_matbuilder->SetSpecularColor(r, g, b);
// }

// void OBJLoader::parse_map_kd_statement() {
//     eat_blanks();

//     const string filename = accept_string();

//     eat_blanks();
//     accept_newline();

//     if(!m_inside_material_def)
//         parse_error();

//     ILuint image_id;

//     ilGenImages(1, &image_id);
//     ilBindImage(image_id);

//     if(!TryLoadingImage(m_path, filename)) {
//         //!\todo Close files and free resources.

//         // There is no distinction between a missing texture file
//         // and an invalid texture file.
//         if(m_option_mask & STOP_ON_MISSING_FILE_BIT) {
//             m_matbuilder->EndMaterial();
//             throw FileNotFoundException(filename);
//         } else return;    // ignore invalid or missing texture file
//     }

//     const int w = ilGetInteger(IL_IMAGE_WIDTH);
//     const int h = ilGetInteger(IL_IMAGE_HEIGHT);

//     unsigned char *texels = new unsigned char[w * h * 3];

//     ilCopyPixels(0, 0, 0, w, h, 1, IL_RGB, IL_UNSIGNED_BYTE, texels);
//     ilDeleteImages(1, &image_id);

//     assert(m_matbuilder);
//     m_matbuilder->SetTexture(w, h, texels);

//     delete [] texels;
// }

// void OBJLoader::parse_newmtl_statement() {
//     eat_blanks();

//     const string material_name = accept_string();

//     eat_blanks();
//     accept_newline();

//     assert(m_matbuilder);

//     if(m_inside_material_def)
//         m_matbuilder->EndMaterial();

// #ifdef VERBOSE
//     cerr << "[OBJLoader] Loading material '" << material_name << "'..." << endl;
// #endif    // VERBOSE

//     m_material_id[material_name] = m_matbuilder->BeginMaterial(material_name);
//     m_inside_material_def = true;
// }
//}; /* namespace raptor_raytracer */
