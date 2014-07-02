RAPTOR_VER = default_ver

# Back up RAPTor
raptor_$(RAPTOR_VER).tar.gz :
	tar -cf raptor_$(RAPTOR_VER).tar $(RAPTOR_HOME)
	gzip raptor_$(RAPTOR_VER).tar

# Clean targets
clean ::
	@( cd networking                         ; make clean     )
	@( cd networking/unit_tests              ; make clean     )
	@( cd networking/regression_tests        ; make clean     )
	@( cd terrain_generator                  ; make clean     )
	@( cd terrain_generator/unit_tests       ; make clean     )
	@( cd terrain_generator/regression_tests ; make clean     )
	@( cd physics_engine                     ; make clean     )
	@( cd physics_engine/unit_tests          ; make clean     )
	@( cd physics_engine/regression_tests    ; make clean     )
	@( cd raytracer                          ; make spotless  )
	@( cd raytracer/unit_tests               ; make spotless  )
	@( cd raytracer/regression_tests         ; make spotless  )
	@( cd games/quake2                       ; make distclean )
