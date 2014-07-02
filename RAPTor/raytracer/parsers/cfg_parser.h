#ifndef __CFG_PARSER_H__
#define __CFG_PARSER_H__

class camera;

void cfg_parser(
    ifstream            &cfg_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);

#endif	/* #ifndef __CFG_PARSER_H__ */
