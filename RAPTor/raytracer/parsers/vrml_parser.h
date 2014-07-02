#ifndef __VRML_PARSER_H__
#define __VRML_PARSER_H__

class camera;

void vrml_parser(
    ifstream            &vrml_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              *c,
    const string        &v);

#endif /* #ifndef __VRML_PARSER_H__ */
