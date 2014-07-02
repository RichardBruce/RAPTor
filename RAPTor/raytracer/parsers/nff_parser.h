#ifndef __NFF_PARSER_H__
#define __NFF_PARSER_H__

class camera;

void nff_parser(
    ifstream            &nff_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m,
    camera              **c);

#endif /* #ifndef __NFF_PARSER_H__ */
