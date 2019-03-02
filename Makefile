.PHONY: install
install:
	idris --install box2d.ipkg

.PHONY: clean
clean:
	idris --clean box2d.ipkg
