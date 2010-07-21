{- This file is part of "settle3D" software.

Copyright (C) 2009, 2010
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

"Settle3D" is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

"Settle3D" is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
"settle3D".  If not, see <http://www.gnu.org/licenses/>.


Author: Dmitrij Yu. Naumov

-}


module Random (module System.Random) where

import System.Random
import Foreign.C.Types (CFloat)

instance Random CFloat where
    randomR (l, h) g = (realToFrac a :: CFloat, g')
        where (a :: Double, g') = randomR (realToFrac l, realToFrac h) g
                        

    random g = (realToFrac a :: CFloat, g')
        where (a :: Double, g') = random g
