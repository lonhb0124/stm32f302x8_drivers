# Makefile in stm32f302x8_drivers



GIT_ADD_PATHS = \
	Makefile \
	.settings \
	drivers \
	Inc \
	Src \
	Startup \
	.cproject \
	.project \
	Makefile \
	STM32F302R8TX_FLASH.ld \
	"stm32f302x8_drivers Debug.launch"

.PHONY: git-add

git-add:
	git add $(GIT_ADD_PATHS)
	@echo "Added: $(GIT_ADD_PATHS)"