# champ

launch > **champ.urdf.xacro**を呼び出す

champ.urdf.xacro は
  <!-- mesh path, mass, and x, y and z length -->
  <xacro:include filename="$(find champ_description)/urdf/properties.urdf.xacro" />
  <!-- legのlink, joint設定をインクルード -->
  <xacro:include filename="$(find champ_description)/urdf/leg.urdf.xacro" />
  <!-- sensor類の設定をインクルード -->
  <xacro:include filename="$(find champ_description)/urdf/accessories.urdf.xacro" />
を呼び出す

xxx_description_xacroにTODOを以下追加
foot_linkの追加、