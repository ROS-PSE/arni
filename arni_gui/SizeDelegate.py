#TODO: imports

class SizeDelegate:QtGui.QStyledItemDelegate(object):
"""Makes it possible to change the font size of the Gui-Plugin content."""
 
    def paint(self, painter, option, index):
	"""Defines how the items of the model will be painted in the view. Can be used  to draw e.g. bigger or smaller fonts

	:param painter:
	:type painter: QPainter
	:param option:
	:type param: QStyleOptionViewItem
	:param index:
	:type index: QModellIndex
	"""
	pass

    def set_bigger_font_size(self):
	"""Increases the displayed font-size"""
	pass

    def set_smaller_font_size(self):
	"""Decreases the displayed font-size"""
	pass
