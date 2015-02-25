#ifndef __LWO_CHUNKS_H__
#define __LWO_CHUNKS_H__

/* Boost headers */

/* Common headers */



namespace raptor_raytracer
{
/* Class to track chunks in current layer */
class lwo_chunks
{
    public :
        lwo_chunks(const char *data, const char *const end) :
            _end(end), _next_layr(data), _tags(nullptr)
        {
            new_layer();
        }

        bool eof()
        {
            return _next_layr >= _end;
        }

        bool populate_globals()
        {
            const char *at = _next_layr;
            while (at < (_end - 4))
            {
                if ((strncmp(at, "TAGS", 4) == 0) && (_tags == nullptr))
                {
                    at += 4;
                    _tags_len = from_byte_stream<std::uint32_t>(&at);
                    _tags = at;

                    BOOST_LOG_TRIVIAL(trace) << "TAGS " << _tags_len;
                    at += _tags_len;
                }
                else if (strncmp(at, "CLIP", 4) == 0)
                {
                    at += 4;
                    const std::uint32_t clip_len = from_byte_stream<std::uint32_t>(&at);
                    _clips.emplace_back(at, clip_len);

                    at += clip_len;
                    BOOST_LOG_TRIVIAL(trace) << "CLIP " << clip_len;
                }
                else if (strncmp(at, "SURF", 4) == 0)
                {
                    at += 4;
                    const std::uint32_t surf_len = from_byte_stream<std::uint32_t>(&at);
                    _surfs.emplace_back(at, surf_len);

                    at += surf_len;
                    BOOST_LOG_TRIVIAL(trace) << "SURF " << surf_len;
                }
                else
                {
                    /* Skip it */
                    at += 4;
                    const std::uint32_t len = from_byte_stream<std::uint32_t>(&at);
                    at += len;
                }
            }

            return (_tags != nullptr) && !_surfs.empty();
        }


        /* Populate chunk pointers for this layer */
        bool populate_layer()
        {
            /* All pointers to nullptr */
            new_layer();

            const char *at = _next_layr + 4;
            _layr_len = from_byte_stream<std::uint32_t>(&at);
            _layr = at;   /* This might not be a layr if file starts with data not in a layr */
            at += _layr_len;
            while (at < _end)
            {
                if (strncmp(at, "LAYR", 4) == 0)
                {
                    BOOST_LOG_TRIVIAL(warning) << "Next LAYR found (not handled)";
                    break;
                }
                else if (strncmp(at, "PNTS", 4) == 0)
                {
                    at += 4;
                    _pnts_len = from_byte_stream<std::uint32_t>(&at);
                    _pnts = at;

                    BOOST_LOG_TRIVIAL(trace) << "PNTS " << _pnts_len;
                    at += _pnts_len;
                }
                else if (strncmp(at, "POLS", 4) == 0)
                {
                    at += 4;
                    _pols_len = from_byte_stream<std::uint32_t>(&at);
                    _pols = at;

                    BOOST_LOG_TRIVIAL(trace) << "POLS " << _pols_len;
                    at += _pols_len;
                }
                else if (strncmp(at, "VMAP", 4) == 0)
                {
                    at += 4;
                    const std::uint32_t vmap_len = from_byte_stream<std::uint32_t>(&at);
                    _vmaps.emplace_back(at, vmap_len);

                    BOOST_LOG_TRIVIAL(trace) << "VMAP " <<vmap_len;
                    at += vmap_len;
                }
                else if (strncmp(at, "PTAG", 4) == 0)
                {
                    at += 4;
                    _ptag_len = from_byte_stream<std::uint32_t>(&at);
                    _ptag = at;

                    BOOST_LOG_TRIVIAL(trace) << "PTAG " << _ptag_len;
                    at += _ptag_len;
                }
                else if (strncmp(at, "VMAD", 4) == 0)
                {
                    at += 4;
                    const std::uint32_t vmad_len = from_byte_stream<std::uint32_t>(&at);
                    _vmads.emplace_back(at, vmad_len);

                    BOOST_LOG_TRIVIAL(trace) << "VMAD " << vmad_len;
                    at += vmad_len;
                }
                else if (strncmp(at, "CLIP", 4) == 0)
                {
                    at = _end;
                    BOOST_LOG_TRIVIAL(trace) << "CLIP found, no more layers";
                    break;
                }
                else if (strncmp(at, "SURF", 4) == 0)
                {
                    at = _end;
                    BOOST_LOG_TRIVIAL(trace) << "SURF found, no more layers";
                    break;
                }
                else if (strncmp(at, "BBOX", 4) == 0)
                {
                    at += 4;
                    _bbox_len = from_byte_stream<std::uint32_t>(&at);
                    _bbox = at;

                    BOOST_LOG_TRIVIAL(info) << "BBOX (ignored) " << _bbox_len;
                    at += _bbox_len;
                }
                else if (strncmp(at, "DESC", 4) == 0)
                {
                    at += 4;
                    _desc_len = from_byte_stream<std::uint32_t>(&at);
                    _desc = at;

                    BOOST_LOG_TRIVIAL(info) << "DESC (ignored) " << _desc_len;
                    at += _desc_len;
                }
                else if (strncmp(at, "TEXT", 4) == 0)
                {
                    at += 4;
                    _text_len = from_byte_stream<std::uint32_t>(&at);
                    _text = at;

                    BOOST_LOG_TRIVIAL(info) << "TEXT (ignored) " << _text_len;
                    at += _text_len;
                }
                else if (strncmp(at, "ICON", 4) == 0)
                {
                    at += 4;
                    _icon_len = from_byte_stream<std::uint32_t>(&at);
                    _icon = at;

                    BOOST_LOG_TRIVIAL(info) << "ICON (ignored) " << _icon_len;
                    at += _icon_len;
                }
            }

            _next_layr = at;
        
            /* Check if we have enough data in the layr */
            return (_pnts != nullptr) && (_pols != nullptr) && (_ptag != nullptr) && !_surfs.empty();
        }

        int             size_of_clips()         const { return _clips.size();       }
        int             size_of_surfs()         const { return _surfs.size();       }
        int             size_of_vmaps()         const { return _vmaps.size();       }
        int             size_of_vmads()         const { return _vmads.size();       }
        const char *    clip(const int i)       const { return _clips[i].first;     }
        const char *    surf(const int i)       const { return _surfs[i].first;     }
        const char *    vmap(const int i)       const { return _vmaps[i].first;     }
        const char *    vmad(const int i)       const { return _vmads[i].first;     }
        std::uint32_t   clip_len(const int i)   const { return _clips[i].second;    }
        std::uint32_t   surf_len(const int i)   const { return _surfs[i].second;    }
        std::uint32_t   vmap_len(const int i)   const { return _vmaps[i].second;    }
        std::uint32_t   vmad_len(const int i)   const { return _vmads[i].second;    }
        
        const char *    layr()      const { return _layr;       }
        const char *    pnts()      const { return _pnts;       }
        const char *    pols()      const { return _pols;       }
        const char *    ptag()      const { return _ptag;       }
        const char *    bbox()      const { return _bbox;       }
        const char *    desc()      const { return _desc;       }
        const char *    text()      const { return _text;       }
        const char *    icon()      const { return _icon;       }
        const char *    tags()      const { return _tags;       }
        std::uint32_t   layr_len()  const { return _layr_len;   }
        std::uint32_t   pnts_len()  const { return _pnts_len;   }
        std::uint32_t   pols_len()  const { return _pols_len;   }
        std::uint32_t   ptag_len()  const { return _ptag_len;   }
        std::uint32_t   bbox_len()  const { return _bbox_len;   }
        std::uint32_t   desc_len()  const { return _desc_len;   }
        std::uint32_t   text_len()  const { return _text_len;   }
        std::uint32_t   icon_len()  const { return _icon_len;   }
        std::uint32_t   tags_len()  const { return _tags_len;   }

    private :
        void new_layer()
        {
            _layr = nullptr;
            _pnts = nullptr;
            _pols = nullptr;
            _ptag = nullptr;
            _bbox = nullptr;
            _desc = nullptr;
            _text = nullptr;
            _icon = nullptr;
        }

        std::vector<std::pair<const char *, int>>   _clips;         /* Start of clips in this layer                                 */
        std::vector<std::pair<const char *, int>>   _surfs;         /* Start of surfaces in this layer                              */
        std::vector<std::pair<const char *, int>>   _vmaps;         /* Vectors associated with vertices in this layer               */
        std::vector<std::pair<const char *, int>>   _vmads;         /* Discontinious vectors associated with vertices in this layer */
        const char *const                           _end;           /* The end of all data                                          */
        const char *                                _next_layr;     /* The start of the next layer                                  */
        const char *                                _layr;          /* The start of this layer                                      */
        const char *                                _pnts;          /* The vertices in this layer                                   */
        const char *                                _pols;          /* The polygons in this layer                                   */
        const char *                                _ptag;          /* */
        const char *                                _bbox;          /* Bounding box of this layer, optional                         */
        const char *                                _desc;          /* Short description of this layer, optional                    */
        const char *                                _text;          /* Long disciption of this layer, optional                      */
        const char *                                _icon;          /* Icon for the file, optional                                  */
        const char *                                _tags;          /* */
        std::uint32_t                               _layr_len;
        std::uint32_t                               _pnts_len;
        std::uint32_t                               _pols_len;
        std::uint32_t                               _ptag_len;
        std::uint32_t                               _bbox_len;
        std::uint32_t                               _desc_len;
        std::uint32_t                               _text_len;
        std::uint32_t                               _icon_len;
        std::uint32_t                               _tags_len;
};
}; /* namespace raptor_raytracer */

#endif /* #ifndef __LWO_CHUNKS_H__ */
