ENTRY(Reset_Handler)

_stack_size = 0x500;

MEMORY {
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
	RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 6K
}

SECTIONS
{
	.text :
	{
		KEEP(*(vectors))
		. = ALIGN(4);
		*(.text)
		*(.text*)
		*(.glue_7)
		*(.glue_7t)
		*(.eh_frame)

		KEEP (*(.init))
		KEEP (*(.fini))

		. = ALIGN(4);
		_etext = .;
	} >FLASH

	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >FLASH

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} >FLASH

	.ARM :
       	{
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >FLASH

	_sidata = LOADADDR(.data);

	.stack (NOLOAD) :
	{
		end = .;
		. = . + _stack_size;
		_estack = .;
	} >RAM

	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} >RAM AT> FLASH

	. = ALIGN(4);
	.bss (NOLOAD) :
	{
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	} >RAM

	/DISCARD/ :
	{
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}
}
