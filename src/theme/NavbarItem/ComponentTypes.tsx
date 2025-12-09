/**
 * Custom Navbar Component Types
 * Adds UserButton to navbar
 */

import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import { UserButton } from '../../components/Auth';

export default {
  ...ComponentTypes,
  'custom-userButton': UserButton,
};
