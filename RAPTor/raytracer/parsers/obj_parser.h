#ifndef __OBJ_PARSER_H__
#define __OBJ_PARSER_H__

class camera;

void obj_parser(
    ifstream            &obj_file,
    string              p,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);

#endif	/* #ifndef __OBJ_PARSER_H__ */
