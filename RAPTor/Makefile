RAPTOR_VER = default_ver

# Back up RAPTor
raptor_$(RAPTOR_VER).tar.gz :
	tar -cf raptor_$(RAPTOR_VER).tar $(RAPTOR_HOME)
	gzip raptor_$(RAPTOR_VER).tar

# Clean targets
clean ::
	$(MAKE) -C networking clean
	$(MAKE) -C terrain_generator clean
	$(MAKE) -C physics_engine clean
	$(MAKE) -C raytracer clean
	@( cd games/quake2 ; make distclean )

spotless ::
	$(MAKE) -C networking spotless
	$(MAKE) -C terrain_generator spotless
	$(MAKE) -C physics_engine spotless
	$(MAKE) -C raytracer spotless
	@( cd games/quake2 ; make distclean )

