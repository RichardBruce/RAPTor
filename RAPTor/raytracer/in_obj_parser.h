// #ifndef __OBJ_PARSER_H__
// #define __OBJ_PARSER_H__

// #include "imeshbuilder.h"
// #include "imeshloader.h"
// #include "keywordtable.h"

// #include <cassert>
// #include <cstdio>
// #include <map>
// #include <string>
// #include <vector>

// class OBJLoader : public IMeshLoader {
// public:
//     struct ExtendedLoadingException : public LoadingException {
//         ExtendedLoadingException(int line) :
//             m_line(line) {}

//         const int m_line;    //!< The line at which the parse error occured.
//     };

//     //! File structure is broken.
//     struct ParsingException : public ExtendedLoadingException {
//         ParsingException(int line) : ExtendedLoadingException(line) {}
//     };

//     //! An invalid face (with less than 3 vertices) has been encountered.
//     struct InvalidFaceException : public ExtendedLoadingException {
//         InvalidFaceException(int line) : ExtendedLoadingException(line) {}
//     };

//     //! A polygon could not be triangulated, because it is either
//     //! self-intersecting or not planar.
//     struct TriangulationException : public ExtendedLoadingException {
//         TriangulationException(int line) : ExtendedLoadingException(line) {}
//     };

//     OBJLoader();

//     virtual void Load(
//         const std::string &filename,
//         IMeshBuilder &builder,
//         int option_mask = DEFAULT_CONFIGURATION_BIT,
//     ) throw(LoadingException);

// private:
//     enum symbol {
//         END_OF_FILE = 256, END_OF_LINE,
//         IDENTIFIER, INTEGER, DOUBLE,

//         //! Keywords.
//         D,            //!< Material transparency (same as TR).
//         F,            //!< Face.
//         G,            //!< Group name.
//         ILLUM,        //!< Illumination model.
//         KA,            //!< Material ambient color.
//         KD,            //!< Material diffuse color.
//         KS,            //!< Material specular color.
//         MAP_KD,        //!< Material diffuse map.
//         MTLLIB,        //!< Material library.
//         NEWMTL,        //!< New material.
//         O,            //!< Object name.
//         S,            //!< Smoothing group.
//         TR,            //!< Material transparency (same as D).
//         USEMTL,        //!< Material name.
//         V,            //!< Geometric vertex.
//         VN,            //!< Vertex normal.
//         VT,            //!< Texture vertex.
//     };

//     KeywordTable m_keyword_table;

//     std::string m_path;

//     IMeshBuilder::IGeometryBuilder *m_geombuilder;
//     IMeshBuilder::IMaterialBuilder *m_matbuilder;

//     int m_option_mask;

//     FILE *m_file;
//     int m_line;

//     typedef std::map<std::string, IMeshBuilder::FeatureId> string_to_feature_id_map;

//     string_to_feature_id_map m_material_id;

//     //! All features are stored in these vectors as the file is parsed.
//     std::vector<Vector3> m_vertices;
//     std::vector<Vector3> m_normals;
//     std::vector<Vector2> m_texcoords;

//     //! This vector is used to avoid inserting duplicate vertices into the current mesh.
//     //! It is cleared each time a new mesh is created.
//     std::map<int, IMeshBuilder::FeatureId> m_global_vertex_id;

//     bool m_create_submesh;        //!< If true, create a new submesh before processing new faces.
//     bool m_set_submesh_material;                    //!< If true, apply the material specified by 'm_material_id' to the new submesh.
//     IMeshBuilder::FeatureId m_submesh_material_id;    //!< Id of the material to apply to the new submesh.
//     bool m_inside_submesh_def;    //!< If true, we are currently in the middle of a submesh definition.
//     bool m_inside_material_def;    //!< If true, we are currently in the middle of a material definition.

//     void parse_error();

//     int look_ahead();

//     void eat_leading_blanks();
//     void eat_blanks();
//     void eat_line();

//     void accept_char(int ref);
//     void accept_newline();
//     int accept_integer();
//     double accept_double();
//     std::string accept_string();

//     bool is_eol();

//     void select_submesh();

//     void parse_file();

//     void parse_f_statement();
//     void parse_mtllib_statement();
//     void parse_usemtl_statement();
//     void parse_v_statement();
//     void parse_vt_statement();
//     void parse_vn_statement();

//     void load_material_file(const std::string &filename);
//     void parse_material_file();

//     void parse_ka_statement();
//     void parse_kd_statement();
//     void parse_ks_statement();
//     void parse_map_kd_statement();
//     void parse_newmtl_statement();
// };

// #endif    /* #ifndef __OBJ_PARSER_H__ */

// ///*
// //    Sheep - A Rigid Body Dynamics Engine
// //    Copyright (C) 2001-2004 Francois Beaune
// //    Contact: beaune@aist.enst.fr
// //
// //    This file is part of Sheep.
// //
// //    Sheep is free software; you can redistribute it and/or modify
// //    it under the terms of the GNU General Public License as published by
// //    the Free Software Foundation; either version 2 of the License, or
// //    (at your option) any later version.
// //
// //    Sheep is distributed in the hope that it will be useful,
// //    but WITHOUT ANY WARRANTY; without even the implied warranty of
// //    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// //    GNU General Public License for more details.
// //
// //    You should have received a copy of the GNU General Public License
// //    along with Sheep; if not, write to the Free Software
// //    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
// //*/
// //
// //#ifndef SHEEP_MESHIO_OBJLOADER_H
// //#define SHEEP_MESHIO_OBJLOADER_H
// //
// //#include "imeshbuilder.h"
// //#include "imeshloader.h"
// //#include "keywordtable.h"
// //
// //#include <cassert>
// //#include <cstdio>
// //#include <map>
// //#include <string>
// //#include <vector>
// //
// //namespace sheep {
// //
// //    class ProgressMonitor;
// //
// //    class OBJLoader : public IMeshLoader {
// //    public:
// //        struct ExtendedLoadingException : public LoadingException {
// //            ExtendedLoadingException(int line) :
// //                m_line(line) {}
// //
// //            const int m_line;    //!< The line at which the parse error occured.
// //        };
// //
// //        //! File structure is broken.
// //        struct ParsingException : public ExtendedLoadingException {
// //            ParsingException(int line) : ExtendedLoadingException(line) {}
// //        };
// //
// //        //! An invalid face (with less than 3 vertices) has been encountered.
// //        struct InvalidFaceException : public ExtendedLoadingException {
// //            InvalidFaceException(int line) : ExtendedLoadingException(line) {}
// //        };
// //
// //        //! A polygon could not be triangulated, because it is either
// //        //! self-intersecting or not planar.
// //        struct TriangulationException : public ExtendedLoadingException {
// //            TriangulationException(int line) : ExtendedLoadingException(line) {}
// //        };
// //
// //        OBJLoader();
// //
// //        virtual void Load(
// //            const std::string &filename,
// //            IMeshBuilder &builder,
// //            int option_mask = DEFAULT_CONFIGURATION_BIT,
// //            ProgressMonitor *progmon = 0
// //        ) throw(LoadingException);
// //
// //    private:
// //        enum symbol {
// //            END_OF_FILE = 256, END_OF_LINE,
// //            IDENTIFIER, INTEGER, DOUBLE,
// //
// //            //! Keywords.
// //            D,            //!< Material transparency (same as TR).
// //            F,            //!< Face.
// //            G,            //!< Group name.
// //            ILLUM,        //!< Illumination model.
// //            KA,            //!< Material ambient color.
// //            KD,            //!< Material diffuse color.
// //            KS,            //!< Material specular color.
// //            MAP_KD,        //!< Material diffuse map.
// //            MTLLIB,        //!< Material library.
// //            NEWMTL,        //!< New material.
// //            O,            //!< Object name.
// //            S,            //!< Smoothing group.
// //            TR,            //!< Material transparency (same as D).
// //            USEMTL,        //!< Material name.
// //            V,            //!< Geometric vertex.
// //            VN,            //!< Vertex normal.
// //            VT,            //!< Texture vertex.
// //        };
// //
// //        KeywordTable m_keyword_table;
// //
// //        std::string m_path;
// //
// //        IMeshBuilder::IGeometryBuilder *m_geombuilder;
// //        IMeshBuilder::IMaterialBuilder *m_matbuilder;
// //
// //        int m_option_mask;
// //
// //        ProgressMonitor *m_progmon;    //!< May be 0.
// //
// //        FILE *m_file;
// //        int m_line;
// //
// //        typedef std::map<std::string, IMeshBuilder::FeatureId> string_to_feature_id_map;
// //
// //        string_to_feature_id_map m_material_id;
// //
// //        //! All features are stored in these vectors as the file is parsed.
// //        std::vector<Vector3> m_vertices;
// //        std::vector<Vector3> m_normals;
// //        std::vector<Vector2> m_texcoords;
// //
// //        //! This vector is used to avoid inserting duplicate vertices into the current mesh.
// //        //! It is cleared each time a new mesh is created.
// //        std::map<int, IMeshBuilder::FeatureId> m_global_vertex_id;
// //
// //        bool m_create_submesh;        //!< If true, create a new submesh before processing new faces.
// //        bool m_set_submesh_material;                    //!< If true, apply the material specified by 'm_material_id' to the new submesh.
// //        IMeshBuilder::FeatureId m_submesh_material_id;    //!< Id of the material to apply to the new submesh.
// //        bool m_inside_submesh_def;    //!< If true, we are currently in the middle of a submesh definition.
// //        bool m_inside_material_def;    //!< If true, we are currently in the middle of a material definition.
// //
// //        void parse_error();
// //
// //        int look_ahead();
// //
// //        void eat_leading_blanks();
// //        void eat_blanks();
// //        void eat_line();
// //
// //        void accept_char(int ref);
// //        void accept_newline();
// //        int accept_integer();
// //        double accept_double();
// //        std::string accept_string();
// //
// //        bool is_eol();
// //
// //        void select_submesh();
// //
// //        void parse_file();
// //
// //        void parse_f_statement();
// //        void parse_mtllib_statement();
// //        void parse_usemtl_statement();
// //        void parse_v_statement();
// //        void parse_vt_statement();
// //        void parse_vn_statement();
// //
// //        void load_material_file(const std::string &filename);
// //        void parse_material_file();
// //
// //        void parse_ka_statement();
// //        void parse_kd_statement();
// //        void parse_ks_statement();
// //        void parse_map_kd_statement();
// //        void parse_newmtl_statement();
// //    };
// //
// //#include "objloader.inl"
// //
// //}
// //
// //#endif    // !SHEEP_MESHIO_OBJLOADER_H
// //